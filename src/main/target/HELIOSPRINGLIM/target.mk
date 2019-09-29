
F405_TARGETS   += $(TARGET)
FEATURES       += VCP ONBOARDFLASH

TARGET_SRC = stm32f4xx_crc.c \
             drivers/dma_spi.c \
						 drivers/barometer/barometer_bmp280.c \
						 drivers/barometer/barometer_ms5611.c \
             drivers/accgyro/accgyro_imuf9001.c \
             drivers/max7456.c
