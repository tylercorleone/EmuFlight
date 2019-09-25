/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "common/axis.h"

#include "pg/pg.h"

extern int16_t safeHold_angle[2];

typedef enum {
    SAFEHOLD_OFF = 0,
    SAFEHOLD_INIT = 1,
    SAFEHOLD_GO_SAFE = 2,
    SAFEHOLD_WAIT = 3,
    SAFEHOLD_LAND = 4,
} safeHoldState_e;

typedef struct volLimitation_s {
    uint8_t altitudeLimitation; // activation of the altitude limitation
    uint8_t distanceLimitation; // activation of the distance limitation
    uint8_t landingAllowed; // Is it allowed to laknd if pilot has not taken control back
    uint16_t maxAltitude; //meters
    uint8_t alertAltitudeBeforeMax; // meters
    uint16_t maxDistance; //meters
    uint8_t alertDistanceBeforeMax; //meters
    uint16_t throttleP, throttleI, throttleD;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint8_t safeHold_pitch_P;
    uint8_t safeHold_pitch_I;
    uint8_t safeHold_pitch_D;
    uint8_t safeHold_roll_P;
    uint8_t safeHold_roll_I;
    uint8_t safeHold_roll_D;
    uint8_t safeHoldAngleMax; // Angle max in deg
    uint8_t minSafeAltitude; // Min altitude for althold in m
    uint8_t minSats;
    uint8_t armingWithoutGps; // possibility to arm without GPS
} volLimitationConfig_t;

PG_DECLARE(volLimitationConfig_t, volLimitationConfig);

typedef struct {
    int32_t currentAltitude;
    uint16_t distanceToHome;
    int16_t directionToHome;
    uint8_t numSat;
    float zVelocity; // Up/down movement in cm/s
    float zVelocityAvg; // Up/down average in cm/s
} volLimSensorData_s;

typedef struct {
    uint8_t altitude;
    uint8_t distance;
    uint8_t safeHold;
    uint8_t sensorFailure;
} volLimAlert_s;

typedef struct {
    uint8_t safeHoldState;
    volLimAlert_s alert;
    volLimSensorData_s sensor;
    uint8_t altitudeLimDemanded;
    uint8_t safeModeDemanded;
    float angleDemand;
    float holdYaw;
} volLimData_s;

void volLimitation_init(void);
void volLimitation_NewGpsData(void);
void volLimitation_SensorUpdate(void);
float volLimitation_AltitudeHold(uint8_t altholdStatus);
uint8_t volLimitation_DistanceLimStatus(void);
void volLimitation_AltitudeLim(void);
 float setAltitudeLim(float throttle);
volLimAlert_s getVolLimAlert(void);
float gpsHoldGetYawRate(void);
bool gpsNeededForVolLim(void);
bool volLimSanityCheck(void);
