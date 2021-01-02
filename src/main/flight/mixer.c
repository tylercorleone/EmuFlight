/*
 * This file is part of Cleanflight and Betaflight and EmuFlight.
 *
 * Cleanflight and Betaflight and EmuFlight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight and EmuFlight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/feature.h"

#include "drivers/dshot.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/pwm_esc_detect.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer_init.h"
#include "flight/mixer_tricopter.h"
#include "flight/pid.h"
#include "flight/rpm_filter.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "mixer.h"

#define DYN_LPF_THROTTLE_UPDATE_DELAY_US 2500 // 2.5ms between updates

static FAST_DATA_ZERO_INIT float controllerMixRange;

float FAST_DATA_ZERO_INIT motor[MAX_SUPPORTED_MOTORS];
float motor_disarmed[MAX_SUPPORTED_MOTORS];

static FAST_DATA_ZERO_INIT int throttleAngleCorrection;

static void twoPassMix(float *motorMix, const float *yawMix, const float *rollPitchMix, float yawMixMin, float yawMixMax,
                       float rollPitchMixMin, float rollPitchMixMax);
static void mixThingsUp(float scaledAxisPidRoll, float scaledAxisPidPitch, float scaledAxisPidYaw, float *motorMix);

float getControllerMixRange(void)
{
    return controllerMixRange;
}

void writeMotors(void)
{
    motorWriteAll(motor);
}

static void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(mixerRuntime.disarmMotorOutput);
    delay(50); // give the timers and ESCs a chance to react.
}

static FAST_DATA_ZERO_INIT float throttle = 0;
static FAST_DATA_ZERO_INIT float mixerThrottle = 0;
static FAST_DATA_ZERO_INIT float motorOutputMin;
static FAST_DATA_ZERO_INIT float motorRangeMin;
static FAST_DATA_ZERO_INIT float motorRangeMax;
static FAST_DATA_ZERO_INIT float motorOutputRange;
static FAST_DATA_ZERO_INIT float motorOutputIdleLevel; // e.g. 5% idle throttle -> 0.05
static FAST_DATA_ZERO_INIT float motorThrustIdleLevel; // corresponding thrust level of motorOutputIdleLevel
static FAST_DATA_ZERO_INIT int8_t controllerMix3DModeSign; // 1 -> normal, -1 -> reversed

static void calculateThrottleAndCurrentMotorEndpoints(timeUs_t currentTimeUs)
{
    static uint16_t rcThrottlePrevious = 0;   // Store the last throttle direction for deadband transitions
    static timeUs_t reversalTimeUs = 0; // time when motors last reversed in 3D mode
    static float motorRangeMinIncrease = 0;

    float currentThrottleInputRange = 0;
    if (mixerRuntime.feature3dEnabled) {
        uint16_t rcCommand3dDeadBandLow;
        uint16_t rcCommand3dDeadBandHigh;

        if (!ARMING_FLAG(ARMED)) {
            rcThrottlePrevious = rxConfig()->midrc; // When disarmed set to mid_rc. It always results in positive direction after arming.
        }

        if (IS_RC_MODE_ACTIVE(BOX3D) || flight3DConfig()->switched_mode3d) {
            // The min_check range is halved because the output throttle is scaled to 500us.
            // So by using half of min_check we maintain the same low-throttle deadband
            // stick travel as normal non-3D mode.
            const int mincheckOffset = (rxConfig()->mincheck - PWM_RANGE_MIN) / 2;
            rcCommand3dDeadBandLow = rxConfig()->midrc - mincheckOffset;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + mincheckOffset;
        } else {
            rcCommand3dDeadBandLow = rxConfig()->midrc - flight3DConfig()->deadband3d_throttle;
            rcCommand3dDeadBandHigh = rxConfig()->midrc + flight3DConfig()->deadband3d_throttle;
        }

        const float rcCommandThrottleRange3dLow = rcCommand3dDeadBandLow - PWM_RANGE_MIN;
        const float rcCommandThrottleRange3dHigh = PWM_RANGE_MAX - rcCommand3dDeadBandHigh;

        if (rcCommand[THROTTLE] <= rcCommand3dDeadBandLow || isFlipOverAfterCrashActive()) {
            // INVERTED
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (controllerMix3DModeSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            controllerMix3DModeSign = -1;

            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand3dDeadBandLow - rcCommand[THROTTLE];
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else if (rcCommand[THROTTLE] >= rcCommand3dDeadBandHigh) {
            // NORMAL
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (controllerMix3DModeSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            controllerMix3DModeSign = 1;
            rcThrottlePrevious = rcCommand[THROTTLE];
            throttle = rcCommand[THROTTLE] - rcCommand3dDeadBandHigh;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        } else if ((rcThrottlePrevious <= rcCommand3dDeadBandLow &&
                !flight3DConfigMutable()->switched_mode3d) ||
                isMotorsReversed()) {
            // INVERTED_TO_DEADBAND
            motorRangeMin = mixerRuntime.motorOutputLow;
            motorRangeMax = mixerRuntime.deadbandMotor3dLow;

#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutputMin = mixerRuntime.motorOutputLow;
                motorOutputRange = mixerRuntime.deadbandMotor3dLow - mixerRuntime.motorOutputLow;
            } else
#endif
            {
                motorOutputMin = mixerRuntime.deadbandMotor3dLow;
                motorOutputRange = mixerRuntime.motorOutputLow - mixerRuntime.deadbandMotor3dLow;
            }

            if (controllerMix3DModeSign != -1) {
                reversalTimeUs = currentTimeUs;
            }
            controllerMix3DModeSign = -1;

            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dLow;
        } else {
            // NORMAL_TO_DEADBAND
            motorRangeMin = mixerRuntime.deadbandMotor3dHigh;
            motorRangeMax = mixerRuntime.motorOutputHigh;
            motorOutputMin = mixerRuntime.deadbandMotor3dHigh;
            motorOutputRange = mixerRuntime.motorOutputHigh - mixerRuntime.deadbandMotor3dHigh;
            if (controllerMix3DModeSign != 1) {
                reversalTimeUs = currentTimeUs;
            }
            controllerMix3DModeSign = 1;
            throttle = 0;
            currentThrottleInputRange = rcCommandThrottleRange3dHigh;
        }
        if (currentTimeUs - reversalTimeUs < mixerRuntime.reverse3dKickTime) {
            // keep iterm zero for 250ms aka 250000 after motor reversal
            // add a motor kick when switching directions
            pidResetIterm();

            float timeFactor = (currentTimeUs - reversalTimeUs) / mixerRuntime.reverse3dKickTime;
            throttle += currentThrottleInputRange * mixerRuntime.reverse3dKick - timeFactor * currentThrottleInputRange * mixerRuntime.reverse3dKick;
        }
    } else {
        throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection;
        currentThrottleInputRange = PWM_RANGE;

#ifdef USE_DYN_IDLE
        if (mixerRuntime.dynIdleMinRps > 0.0f) {
            const float maxIncrease = isAirmodeActivated() ? mixerRuntime.dynIdleMaxIncrease : 0.04f;
            float minRps = rpmMinMotorFrequency();
            DEBUG_SET(DEBUG_DYN_IDLE, 3, (minRps * 10));
            float rpsError = mixerRuntime.dynIdleMinRps - minRps;
            // PT1 type lowpass delay and smoothing for D
            minRps = mixerRuntime.prevMinRps + mixerRuntime.minRpsDelayK * (minRps - mixerRuntime.prevMinRps);
            float dynIdleD = (mixerRuntime.prevMinRps - minRps) * mixerRuntime.dynIdleDGain;
            mixerRuntime.prevMinRps = minRps;
            float dynIdleP = rpsError * mixerRuntime.dynIdlePGain;
            rpsError = MAX(-0.1f, rpsError); //I rises fast, falls slowly
            mixerRuntime.dynIdleI += rpsError * mixerRuntime.dynIdleIGain;
            mixerRuntime.dynIdleI = constrainf(mixerRuntime.dynIdleI, 0.0f, maxIncrease);
            motorRangeMinIncrease = constrainf((dynIdleP + mixerRuntime.dynIdleI + dynIdleD), 0.0f, maxIncrease);

            DEBUG_SET(DEBUG_DYN_IDLE, 0, (MAX(-1000.0f, dynIdleP * 10000)));
            DEBUG_SET(DEBUG_DYN_IDLE, 1, (mixerRuntime.dynIdleI * 10000));
            DEBUG_SET(DEBUG_DYN_IDLE, 2, (dynIdleD * 10000));
       } else {
            motorRangeMinIncrease = 0;
        }
#endif

#if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
        float motorRangeAttenuationFactor = 0;
        // reduce motorRangeMax when battery is full
        if (mixerRuntime.vbatSagCompensationFactor > 0.0f) {
            const uint16_t currentCellVoltage = getBatterySagCellVoltage();
            // batteryGoodness = 1 when voltage is above vbatFull, and 0 when voltage is below vbatLow
            float batteryGoodness = 1.0f - constrainf((mixerRuntime.vbatFull - currentCellVoltage) / mixerRuntime.vbatRangeToCompensate, 0.0f, 1.0f);
            motorRangeAttenuationFactor = (mixerRuntime.vbatRangeToCompensate / mixerRuntime.vbatFull) * batteryGoodness * mixerRuntime.vbatSagCompensationFactor;
            DEBUG_SET(DEBUG_BATTERY, 2, batteryGoodness * 100);
            DEBUG_SET(DEBUG_BATTERY, 3, motorRangeAttenuationFactor * 1000);
        }
        motorRangeMax = mixerRuntime.motorOutputHigh - motorRangeAttenuationFactor * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
#else
        motorRangeMax = mixerRuntime.motorOutputHigh;
#endif

        motorRangeMin = mixerRuntime.motorOutputLow + motorRangeMinIncrease * (mixerRuntime.motorOutputHigh - mixerRuntime.motorOutputLow);
        motorOutputMin = motorRangeMin;
        motorOutputRange = motorRangeMax - motorRangeMin;
        controllerMix3DModeSign = 1;
    }

    motorOutputIdleLevel = ABS((mixerRuntime.motorOutputLow - mixerRuntime.disarmMotorOutput) / (mixerRuntime.motorOutputHigh - mixerRuntime.disarmMotorOutput));
    motorThrustIdleLevel = motorToThrust(motorOutputIdleLevel, false);

    throttle = constrainf(throttle / currentThrottleInputRange, 0.0f, 1.0f);
}

#define CRASH_FLIP_DEADBAND 20
#define CRASH_FLIP_STICK_MINF 0.15f

static void applyFlipOverAfterCrashModeToMotors(void)
{
    if (ARMING_FLAG(ARMED)) {
        const float flipPowerFactor = 1.0f - mixerConfig()->crashflip_expo / 100.0f;
        const float stickDeflectionPitchAbs = getRcDeflectionAbs(FD_PITCH);
        const float stickDeflectionRollAbs = getRcDeflectionAbs(FD_ROLL);
        const float stickDeflectionYawAbs = getRcDeflectionAbs(FD_YAW);

        const float stickDeflectionPitchExpo = flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        const float stickDeflectionRollExpo = flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        const float stickDeflectionYawExpo = flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

        float signPitch = getRcDeflection(FD_PITCH) < 0 ? 1 : -1;
        float signRoll = getRcDeflection(FD_ROLL) < 0 ? 1 : -1;
        float signYaw = (getRcDeflection(FD_YAW) < 0 ? 1 : -1) * (mixerConfig()->yaw_motors_reversed ? 1 : -1);

        float stickDeflectionLength = sqrtf(sq(stickDeflectionPitchAbs) + sq(stickDeflectionRollAbs));
        float stickDeflectionExpoLength = sqrtf(sq(stickDeflectionPitchExpo) + sq(stickDeflectionRollExpo));

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) / (sqrtf(2.0f) * stickDeflectionLength) : 0;
        const float cosThreshold = sqrtf(3.0f)/2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float crashFlipStickMinExpo = flipPowerFactor * CRASH_FLIP_STICK_MINF + power3(CRASH_FLIP_STICK_MINF) * (1 - flipPowerFactor);
        const float flipStickRange = 1.0f - crashFlipStickMinExpo;
        const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

        for (int i = 0; i < mixerRuntime.motorCount; ++i) {
            float motorOutputNormalised =
                signPitch * mixerRuntime.currentMixer[i].pitch +
                signRoll * mixerRuntime.currentMixer[i].roll +
                signYaw * mixerRuntime.currentMixer[i].yaw;

            if (motorOutputNormalised < 0) {
                if (mixerConfig()->crashflip_motor_percent > 0) {
                    motorOutputNormalised = -motorOutputNormalised * (float)mixerConfig()->crashflip_motor_percent / 100.0f;
                } else {
                    motorOutputNormalised = 0;
                }
            }
            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);
            float motorOutput = motorOutputMin + motorOutputNormalised * motorOutputRange;

            // Add a little bit to the motorOutputMin so props aren't spinning when sticks are centered
            motorOutput = (motorOutput < motorOutputMin + CRASH_FLIP_DEADBAND) ? mixerRuntime.disarmMotorOutput : (motorOutput - CRASH_FLIP_DEADBAND);

            motor[i] = motorOutput;
        }
    } else {
        // Disarmed mode
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static void applyMixToMotors(float motorMix[MAX_SUPPORTED_MOTORS])
{
    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float motorOutput = controllerMix3DModeSign * motorMix[i];
#ifdef USE_THRUST_LINEARIZATION
        motorOutput = pidApplyThrustLinearization(motorOutput);
#endif
        motorOutput = motorOutputMin + motorOutputRange * motorOutput;

#ifdef USE_SERVOS
        if (mixerIsTricopter()) {
            motorOutput += mixerTricopterMotorCorrection(i);
        }
#endif
        if (failsafeIsActive()) {
#ifdef USE_DSHOT
            if (isMotorProtocolDshot()) {
                motorOutput = (motorOutput < motorRangeMin) ? mixerRuntime.disarmMotorOutput : motorOutput; // Prevent getting into special reserved range
            }
#endif
            motorOutput = constrainf(motorOutput, mixerRuntime.disarmMotorOutput, motorRangeMax);
        } else {
            motorOutput = constrainf(motorOutput, motorRangeMin, motorRangeMax);
        }
        motor[i] = motorOutput;
    }

    // Disarmed mode
    if (!ARMING_FLAG(ARMED)) {
        for (int i = 0; i < mixerRuntime.motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
}

static float applyThrottleLimit(float throttle)
{
    if (currentControlRateProfile->throttle_limit_percent < 100) {
        const float throttleLimitFactor = currentControlRateProfile->throttle_limit_percent / 100.0f;
        switch (currentControlRateProfile->throttle_limit_type) {
            case THROTTLE_LIMIT_TYPE_SCALE:
                return throttle * throttleLimitFactor;
            case THROTTLE_LIMIT_TYPE_CLIP:
                return MIN(throttle, throttleLimitFactor);
        }
    }

    return throttle;
}

static void applyMotorStop(void)
{
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motor[i] = mixerRuntime.disarmMotorOutput;
    }
}

#ifdef USE_DYN_LPF
static FAST_DATA_ZERO_INIT float dynLpf2Cutoff[XYZ_AXIS_COUNT];

static void updateDynLpf2(int axis) {
  float gyroFiltered = gyro.gyroADCf[axis];
  float target = getSetpointRate(axis);
  float error = fabsf(target - gyroFiltered);

  float average = (fabsf(target) + fabsf(gyroFiltered)) * 0.5f;
  average = MAX(average, 10.0f);
  getSetpointRate(axis);

  float e = MIN(error / average, 1.0f);                           //Compute ratio between Error and average. e is image of noise in % of signal

  //New freq
  float ratioRatio = MIN(gyroFiltered * gyroFiltered / 200.0f, 1.0f);
  float inverseRatioRatio = 1.0f - ratioRatio;
  dynLpf2Cutoff[axis] = 100.0f * powf(e, 3.0f) * ratioRatio;  //"e" power 3 and multiply by a gain
  dynLpf2Cutoff[axis] += error * inverseRatioRatio / 20.0f;
}

static void updateDynLpfCutoffs(timeUs_t currentTimeUs, float throttle)
{
    static timeUs_t lastDynLpfUpdateUs = 0;
    static float lastGyroCutoff[XYZ_AXIS_COUNT], lastDtermCutoff[XYZ_AXIS_COUNT] = {0,0,0};
    static float gyroCutoff[XYZ_AXIS_COUNT], dtermCutoff[XYZ_AXIS_COUNT] = {0,0,0};
    static float gyroThrottleCutoff, dtermThrottleCutoff;

    if (cmpTimeUs(currentTimeUs, lastDynLpfUpdateUs) >= DYN_LPF_THROTTLE_UPDATE_DELAY_US) {
        const int quantizedThrottle = throttle; // quantize the throttle reduce the number of filter updates
        gyroThrottleCutoff = dynLpfGyroThrCut(quantizedThrottle);
        dtermThrottleCutoff = dynLpfDtermThrCut(quantizedThrottle);


        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            updateDynLpf2(i);

            lastGyroCutoff[i] = dynLpfGyroCutoff(gyroThrottleCutoff, dynLpf2Cutoff[i]);
            lastDtermCutoff[i] = dynLpfDtermCutoff(dtermThrottleCutoff, dynLpf2Cutoff[i]);
            gyroCutoff[i] = pt1FilterApply(&mixerRuntime.dynGyroFc[i], lastGyroCutoff[i]);
            dtermCutoff[i] = pt1FilterApply(&mixerRuntime.dynDtermFc[i], lastDtermCutoff[i]);

            dynLpfGyroUpdate(gyroCutoff);
            dynLpfDTermUpdate(dtermCutoff);

            if (i == FD_ROLL) {
                DEBUG_SET(DEBUG_DYN_LPF, 0, gyroThrottleCutoff);
                DEBUG_SET(DEBUG_DYN_LPF, 1, gyroCutoff[i]);
                DEBUG_SET(DEBUG_DYN_LPF, 2, dtermThrottleCutoff);
                DEBUG_SET(DEBUG_DYN_LPF, 3, dtermCutoff[i]);
            }
        }

        lastDynLpfUpdateUs = currentTimeUs;
    }
}
#endif

void mixWithThrottleLegacy(float *motorMix, float *controllerMix, float controllerMixMin, float controllerMixMax) {
    float throttleThrust = currentPidProfile->linear_throttle ? throttle : motorToThrust(throttle, true);
    float normFactor = 1 / (controllerMixRange > 1.0f ? controllerMixRange : 1.0f);

    if (currentPidProfile->mixer_impl == MIXER_IMPL_LEGACY) {
        // legacy clipping handling
        if (normFactor < 1.0f) {
            for (int i = 0; i < mixerRuntime.motorCount; i++) {
                controllerMix[i] *= normFactor;
            }
            // Get the maximum correction by setting offset to center when airmode enabled
            if (airmodeIsEnabled()) {
                throttleThrust = 0.5f;
            }
        } else {
            if (airmodeIsEnabled() || throttleThrust > 0.5f) {  // Only automatically adjust throttle when airmode enabled. Airmode logic is always active on high throttle
                throttleThrust = constrainf(throttleThrust, -controllerMixMin, 1.0f - controllerMixMax);
            }
        }
    } else {
        float throttleMotor = currentPidProfile->linear_throttle ? thrustToMotor(throttle, true) : throttle; // used to make authority grow faster
        float authority = airmodeIsEnabled() ? 1.0f : SCALE_UNITARY_RANGE(throttleMotor, 0.5f, 1.0f);
        normFactor *= authority;
    }

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        if (currentPidProfile->mixer_impl == MIXER_IMPL_SMOOTH) {
            // clipping handling
            float offset = mixerRuntime.mixerLaziness ? (ABS(controllerMix[i]) * SCALE_UNITARY_RANGE(throttleThrust, 1, -1))
                                         : SCALE_UNITARY_RANGE(throttleThrust, -controllerMixMin, -controllerMixMax);
            controllerMix[i] = (controllerMix[i] + offset) * normFactor;
        }
        float thrustMix = controllerMix[i] + throttleThrust * mixerRuntime.currentMixer[i].throttle;
        motorMix[i] = thrustToMotor(thrustMix, true);
    }
}

FAST_CODE_NOINLINE void mixTable(timeUs_t currentTimeUs)
{

    // Find min and max throttle based on conditions. Throttle has to be known before mixing
    calculateThrottleAndCurrentMotorEndpoints(currentTimeUs);

    if (isFlipOverAfterCrashActive()) {
        applyFlipOverAfterCrashModeToMotors();

        return;
    }

    // Calculate and Limit the PID sum
    const float scaledAxisPidRoll =
            constrainf(pidData[FD_ROLL].Sum * mixerRuntime.linearThrustPIDScaler, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;
    const float scaledAxisPidPitch =
        constrainf(pidData[FD_PITCH].Sum * mixerRuntime.linearThrustPIDScaler, -currentPidProfile->pidSumLimit, currentPidProfile->pidSumLimit) / PID_MIXER_SCALING;

    uint16_t yawPidSumLimit = currentPidProfile->pidSumLimitYaw;

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinDetected = gyroYawSpinDetected();
    if (yawSpinDetected) {
        yawPidSumLimit = PIDSUM_LIMIT_MAX;   // Set to the maximum limit during yaw spin recovery to prevent limiting motor authority
    }
#endif // USE_YAW_SPIN_RECOVERY

    float scaledAxisPidYaw =
        constrainf(pidData[FD_YAW].Sum * mixerRuntime.linearThrustYawPIDScaler, -yawPidSumLimit, yawPidSumLimit) / PID_MIXER_SCALING;

    if (!mixerConfig()->yaw_motors_reversed) {
        scaledAxisPidYaw = -scaledAxisPidYaw;
    }

    // Apply the throttle_limit_percent to scale or limit the throttle based on throttle_limit_type
    if (currentControlRateProfile->throttle_limit_type != THROTTLE_LIMIT_TYPE_OFF) {
        throttle = applyThrottleLimit(throttle);
    }

    // use scaled throttle, without dynamic idle throttle offset, as the input to antigravity
    pidUpdateAntiGravityThrottleFilter(throttle);

#ifdef USE_DYN_LPF
    // keep the changes to dynamic lowpass clean, without unnecessary dynamic changes
    updateDynLpfCutoffs(currentTimeUs, throttle);
#endif

    // apply throttle boost when throttle moves quickly
#if defined(USE_THROTTLE_BOOST)
    if (throttleBoost > 0.0f) {
        const float throttleHpf = throttle - pt1FilterApply(&throttleLpf, throttle);
        throttle = constrainf(throttle + throttleBoost * throttleHpf, 0.0f, 1.0f);
    }
#endif

    // send throttle value to blackbox, including scaling and throttle boost, but not TL compensation, dyn idle or airmode
    mixerThrottle = throttle;

#ifdef USE_DYN_IDLE
    // Apply digital idle throttle offset when stick is at zero after all other adjustments are complete
    if (mixerRuntime.dynIdleMinRps > 0.0f) {
        throttle = MAX(throttle, mixerRuntime.idleThrottleOffset);
    }
#endif

#ifdef USE_THRUST_LINEARIZATION
    // reduce throttle to offset additional motor output
    throttle = pidCompensateThrustLinearization(throttle);
#endif

    //  The following fixed throttle values will not be shown in the blackbox log
    // ?? Should they be influenced by airmode?  If not, should go after the apply airmode code.
#ifdef USE_YAW_SPIN_RECOVERY
    // 50% throttle provides the maximum authority for yaw recovery when airmode is not active.
    // When airmode is active the throttle setting doesn't impact recovery authority.
    if (yawSpinDetected && !airmodeIsEnabled() && !isLaunchControlActive()) {
        throttle = 0.5f;
    }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
    // While launch control is active keep the throttle at minimum.
    // Once the pilot triggers the launch throttle control will be reactivated.
    if (isLaunchControlActive()) {
        throttle = 0.0f;
    }
#endif

#ifdef USE_GPS_RESCUE
    // If gps rescue is active then override the throttle. This prevents things
    // like throttle boost or throttle limit from negatively affecting the throttle.
    if (FLIGHT_MODE(GPS_RESCUE_MODE)) {
        throttle = gpsRescueGetThrottle();
    }
#endif

    float motorMix[MAX_SUPPORTED_MOTORS];

    // mix controller output with throttle
    mixThingsUp(scaledAxisPidRoll, scaledAxisPidPitch, scaledAxisPidYaw, motorMix);

    if (featureIsEnabled(FEATURE_MOTOR_STOP)
        && ARMING_FLAG(ARMED)
        && !mixerRuntime.feature3dEnabled
        && !airmodeIsEnabled()
        && !isLaunchControlActive()
        && !FLIGHT_MODE(GPS_RESCUE_MODE)   // disable motor_stop while GPS Rescue is active
        && (rcData[THROTTLE] < rxConfig()->mincheck)) {
        // motor_stop handling
        applyMotorStop();
    } else {
        // Apply the mix to motor endpoints
        applyMixToMotors(motorMix);
    }
}

void mixThingsUp(const float scaledAxisPidRoll, const float scaledAxisPidPitch, const float scaledAxisPidYaw, float *motorMix) {
    float yawMix[MAX_SUPPORTED_MOTORS];
    float rollPitchMix[MAX_SUPPORTED_MOTORS];
    float controllerMix[MAX_SUPPORTED_MOTORS];
    float yawMixMin = 0, yawMixMax = 0;
    float rollPitchMixMin = 0, rollPitchMixMax = 0;
    float controllerMixMin = 0, controllerMixMax = 0;

    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        float yawMixVal = controllerMix3DModeSign * scaledAxisPidYaw * mixerRuntime.currentMixer[i].yaw;
        if (yawMixVal > yawMixMax) {
            yawMixMax = yawMixVal;
        } else if (yawMixVal < yawMixMin) {
            yawMixMin = yawMixVal;
        }
        yawMix[i] = yawMixVal;

        float rollPitchMixVal = scaledAxisPidRoll * mixerRuntime.currentMixer[i].roll + scaledAxisPidPitch * mixerRuntime.currentMixer[i].pitch;
        if (rollPitchMixVal > rollPitchMixMax) {
            rollPitchMixMax = rollPitchMixVal;
        } else if (rollPitchMixVal < rollPitchMixMin) {
            rollPitchMixMin = rollPitchMixVal;
        }
        rollPitchMix[i] = rollPitchMixVal;

        float controllerMixVal = controllerMix3DModeSign * (rollPitchMixVal + yawMixVal);
        if (controllerMixVal > controllerMixMax) {
            controllerMixMax = controllerMixVal;
        } else if (controllerMixVal < controllerMixMin) {
            controllerMixMin = controllerMixVal;
        }
        controllerMix[i] = controllerMixVal;
    }

    controllerMixRange = controllerMixMax - controllerMixMin; // measures how much the controller is trying to compensate

    if (currentPidProfile->mixer_impl == MIXER_IMPL_2PASS) {
        twoPassMix(motorMix, yawMix, rollPitchMix, yawMixMin, yawMixMax, rollPitchMixMin, rollPitchMixMax);
    } else {
        mixWithThrottleLegacy(motorMix, controllerMix, controllerMixMin, controllerMixMax);
    }
}

void mixerSetThrottleAngleCorrection(int correctionValue)
{
    throttleAngleCorrection = correctionValue;
}

float mixerGetThrottle(void)
{
    return mixerThrottle;
}

float thrustToMotor(float thrust, bool fromIdleLevelOffset) {
    if (!mixerRuntime.linearThrustEnabled) {
        return thrust;
    }

    thrust = constrainf(thrust, 0.0f, 1.0f);

    if (fromIdleLevelOffset) {
        // simply applying some shifts to the graph, for more info see https://www.desmos.com/calculator/lgtopxo5mt
        float x = thrustToMotor(thrust * (1.0f - motorThrustIdleLevel) + motorThrustIdleLevel , false);
        return (x - motorOutputIdleLevel) / (1.0f - motorOutputIdleLevel);
    }

    float compLevel = SCALE_UNITARY_RANGE(thrust, mixerRuntime.linearThrustLowOutput, mixerRuntime.linearThrustHighOutput);
    return (compLevel - 1 + sqrtf(sq(1.0f - compLevel) + 4.0f * compLevel * thrust)) / (2.0f * compLevel);
}

float motorToThrust(float motor, bool fromIdleLevelOffset) {
    if (!mixerRuntime.linearThrustEnabled) {
        return motor;
    }

    motor = constrainf(motor, 0.0f, 1.0f);

    if (fromIdleLevelOffset) {
        // simply applying some shifts to the graph, for more info see https://www.desmos.com/calculator/lgtopxo5mt
        float x = motorToThrust(motor * (1.0f - motorOutputIdleLevel) + motorOutputIdleLevel , false);
        return (x - motorThrustIdleLevel) / (1.0f - motorThrustIdleLevel);
    }

    float compLevel = SCALE_UNITARY_RANGE(motor, mixerRuntime.linearThrustLowOutput, mixerRuntime.linearThrustHighOutput);
    return (1.0f - compLevel) * motor + compLevel * sq(motor);
}

static void twoPassMix(float *motorMix, const float *yawMix, const float *rollPitchMix, float yawMixMin, float yawMixMax,
                float rollPitchMixMin, float rollPitchMixMax) {

    float throttleMotor = currentPidProfile->linear_throttle ? thrustToMotor(throttle, true) : throttle;
    float authority = airmodeIsEnabled() ? 1.0f : SCALE_UNITARY_RANGE(throttleMotor, 0.5f, 1.0f);

    float controllerMixNormFactor = authority / MAX(controllerMixRange, 1.0f);

    // filling up motorMix with throttle, yaw and roll/pitch
    for (int i = 0; i < mixerRuntime.motorCount; i++) {
        motorMix[i] = throttleMotor; // motorMix have to contain output-proportional values

        // clipping handling
        float yawOffset = mixerRuntime.mixerLaziness ? (ABS(yawMix[i]) * SCALE_UNITARY_RANGE(throttleMotor, 1, -1))
                : SCALE_UNITARY_RANGE(throttleMotor, -yawMixMin, -yawMixMax);

        motorMix[i] += (yawMix[i] + yawOffset) * controllerMixNormFactor; // yaw is an output-proportional value (RPM-proportional, actually)

        float motorMixThrust = motorToThrust(motorMix[i], true); // convert into thrust value

        // clipping handling
        float rollPitchOffset = mixerRuntime.mixerLaziness ? (ABS(rollPitchMix[i]) * SCALE_UNITARY_RANGE(motorMixThrust, 1, -1))
                : SCALE_UNITARY_RANGE(motorMixThrust, -rollPitchMixMin, -rollPitchMixMax);

        motorMixThrust += (rollPitchOffset + rollPitchMix[i]) * controllerMixNormFactor; // roll and pitch are thrust-proportional values

        motorMix[i] = thrustToMotor(motorMixThrust, true); // translating back into motor value
    }
}
