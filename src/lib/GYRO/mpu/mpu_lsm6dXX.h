#pragma once

#include "stdint.h"
#include "mpu.h"

// See INAV src/main/drivers/accgyro/accgyro_lms6dxx.c


class MPUDev_LSM6DXX : public MPU_Base
{
    using MPU_Base::MPU_Base;
    public:
        const char *GetMPUName();
        bool initialize();
        void start();
        bool read(float accel_rpy[], float angle_rpy[]);
        void calibrate(bool save);

    protected:
        bool rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
    private:
       
        
        #ifdef DEBUG_GYRO_STATS
        void print_gyro_stats();
        unsigned long last_gyro_stats_time;
        #endif
};
