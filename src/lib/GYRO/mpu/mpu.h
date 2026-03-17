#pragma once

#include "helper_3dmath.h"
#include "gyro_types.h"

class MPU_Base
{
    public:
        static uint8_t m_address;

        virtual bool initialize();
        
        virtual void start();
        virtual uint8_t event();
        virtual bool read(float accel_rpy[], float angle_rpy[]);
       
        void calibrate(bool save);
        virtual void OrientationHorizontalExecute();
        virtual void OrientationVerticalExecute();
        virtual bool isRunning();

        void setupOrientation();
        void applyOrientation(VectorInt16 *v);
        void applyOrientation(Quaternion *q);

        void findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx);
        uint8_t readAndGetGravity();

        // I2C Read/Write
        bool readRegister(uint8_t reg, uint8_t *data, size_t size);
        void writeRegister(uint8_t reg, uint8_t value);
        void writeRegisterBits(uint8_t registerID, uint8_t mask, uint8_t value);

    protected:
        unsigned long last_gyro_update;

        // Orientation related variables
        bool orientationIsWrong;    // flag to say that orientation is wrong and so avoid any process of raw data
        uint8_t mpuOrientationH=0;
        uint8_t mpuOrientationV=0;

        uint8_t orientationX;       // contain the index 0,1, 2 of aRaw[] and gRaw[] to be moved in oax and ogx
        uint8_t orientationY;       // idem for oay and ogy
        uint8_t orientationZ;       // idem for oaz and ogz
        int8_t orientationSignX;    // contains the sign (1 or -1 ) to apply to oax and ogx
        int8_t orientationSignY;    // idem for oay and ogy
        int8_t orientationSignZ;    // idem for oaz and ogz

        rx_config_gyro_calibration_t cal_gyro_offsets, cal_accel_offets;

        float   accScale1G, gyroScaleRad, gyroScaleDeg;

        Quaternion  q = Quaternion();        // [w, x, y, z]         quaternion container
        VectorInt16 v_gyro, v_accel;
        VectorFloat gravity; // [x, y, z]            gravity vector    
        //float euler[3];      // [psi, theta, phi]    Euler angle container
        float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container

        virtual bool rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
        void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat, Quaternion *q);

        virtual bool CalibrateGyro(int8_t loops, rx_config_gyro_calibration_t *offsets);
        virtual bool CalibrateAccel(int8_t loops, rx_config_gyro_calibration_t *offsets);

        void GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
        uint8_t GetGravity(VectorFloat *v, Quaternion *q);

        #ifdef DEBUG_GYRO_STATS
        void print_gyro_stats();
        #endif
        
};