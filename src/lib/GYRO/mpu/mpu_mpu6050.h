#pragma once

#include "stdint.h"
#include "mpu.h"
#include "MPU6050_6Axis_MotionApps612.h"

// See betaflight src/main/drivers/accgyro/accgyro_mpu6050.c

class MPUDev_MPU6050 : public MPU_Base
{
    using MPU_Base::MPU_Base;
    public:
        const char *GetMPUName();
        bool initialize();
        void start();

    protected:
        bool rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
    private:
        MPU6050 *mpu;
};
