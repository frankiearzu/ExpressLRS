#include <Arduino.h>
#include "MPU6050.h"

#define WHO_AM_I 0x75
#define ACCEL_X_H 0x3B

#define ARES (16.0 / 32768)
#define GRES (2000.0 / 32768)

bool MPU6050::initialize() {
    // First check if this an MPU6050
    if (((readRegister(WHO_AM_I) >> 1) & 0x3F) != 0x34) {
        return false;
    }

    writeRegister(0x6B, 0x01);  // PWR_MGMT_1, use X axis gyro as clock reference
    writeRegister(0x1B, 0x18);  // GYRO_CONFIG, 2000dps
    writeRegister(0x1C, 0x18);  // ACCEL_CONFIG, +-16g
    writeRegister(0x1A, 0x01);  // CONFIG, 184/188Hz DLPF
    writeRegister(0x19, 0x09);  // SMPRT_DIV, 1kHz / (1 + 9) = 100Hz
    writeRegister(0x37, 0x02);  // INT_PIN_CFG, I2C_BYPASS_EN
    writeRegister(0x38, 0x01);  // INT_ENABLE, DATA_RDY_EN
    writeRegister(0x23, 0x00);  // FIFO_EN, disabled

    gyroRange = 2000.0;
  
    return true;
}

bool MPU6050::getRawData(int16_t *ax, int16_t *ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t *gz) {
    uint8_t values[14];

    // Read Accel, temp & gyro data (ignore the temp)
    readRegistersBuffer(ACCEL_X_H, values, 14);
    *ax =  (int16_t)((values[0] << 8) | values[1]) * ARES;
    *ay =  (int16_t)((values[2] << 8) | values[3]) * ARES;
    *az =  (int16_t)((values[4] << 8) | values[5]) * ARES;
    *gx =  (int16_t)((values[8] << 8) | values[9]) * GRES;
    *gy =  (int16_t)((values[10] << 8) | values[11]) * GRES;
    *gz =  (int16_t)((values[12] << 8) | values[13]) * GRES;
    return true;
}
