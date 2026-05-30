#pragma once
#include "FusionMath.h"
#include "IMUBase.h"

class MPU6050 : public IMUBase {
public:
    MPU6050() : IMUBase(0x68) {}

    bool initialize();
    bool getRawData(int16_t *ax, int16_t *ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t *gz);

protected:
};