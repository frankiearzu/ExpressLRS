#include "targets.h"

#if defined(HAS_GYRO) && defined(GYRO_DEVICE_MPU6050)
#include "mpu_mpu6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "logging.h"
#include "config.h"

#define I2C_MASTER_FREQ_HZ 400000

static uint8_t accScaleCode, gyroScaleCode;


const char * MPUDev_MPU6050::GetMPUName() {
    return "MPU6050";
}

bool MPUDev_MPU6050::initialize() {
    MPU_Base::initialize();
    m_address = MPU6050_DEFAULT_ADDRESS;     // Defaults is MPU6050_ADDRESS_AD0_LOW (0x68)
   
    DBGLN("Detecting MPU6050");
    mpu =  new MPU6050(m_address);
    I2Cdev::readTimeout = 1; // 1ms timeout instead of 1000ms (1s)

    bool found = false;
    for (int8_t i=0;i<5;i++) {
        if (mpu->testConnection()) 
        {
            found = true;
            break;
        }
        delay(50);
    }

    if (!found) {
        mpu = nullptr;
        DBGLN("Detecting MPU6050 (Alt Address)");
        m_address = MPU6050_ADDRESS_AD0_HIGH;    // Use the alternate address (0x69)
        mpu =  new MPU6050(m_address);
        I2Cdev::readTimeout = 1; // 1ms timeout instead of 1000ms (1s)

        found = false;
        for (int8_t i=0;i<5;i++) {
            if (mpu->testConnection()) 
            {
                found = true;
                break;
            }
            delay(50);
        }
    }

    if (!found) 
    {
        DBGLN("MPU6050 not found!");
        mpu = nullptr;
        return false;
    }

    accScaleCode = MPU6050_ACCEL_FS_2; // Acceleation 2G
    accScale1G = 16384.0;              // +/-  32768/2 

    // Data in Centi Degres/sec  
    // Deg/sec = (ADC* (1/16.4)=0.0609756)
    gyroScaleCode = MPU6050_GYRO_FS_2000;  
    gyroScaleRad = 2000.0 / 32768.0 / 180 * PI;  //   multiply adc by this to get rad°/s
    gyroScaleDeg = 2000.0 / 32768.0 / 100;       //   multiply adc by this to get deg°/s  

    orientationIsWrong = true;
    return true;
}

void MPUDev_MPU6050::start() {
    DBGLN("MPU6050 Start");
    
    mpu->reset();
    vTaskDelay(50 * portTICK_PERIOD_MS);

    mpu->initialize();

    mpu->setFullScaleAccelRange(accScaleCode);
    mpu->setFullScaleGyroRange(gyroScaleCode);
    mpu->setMasterClockSpeed(MPU6050_CLOCK_DIV_400); // 400kHz that matches Wire Clock

    //Set frequency filters
    //mpu->setDLPFMode(MPU6050_DLPF_BW_256);   // LPF (Default is 256Hz, 8khz sample rate, delay < 1ms)
    //mpu->setDHPFMode(MPU6050_DHPF_RESET);    // HBPF: (Default, No high filter)
    
    memcpy(&cal_accel_offets,config.GetAccelCalibration(),sizeof(rx_config_gyro_calibration_t));
    memcpy(&cal_gyro_offsets,config.GetGyroCalibration(),sizeof(rx_config_gyro_calibration_t));
    DBGLN("MPU6050 Acc Offs:  x=%d,y=%d,z=%d",cal_accel_offets.x, cal_accel_offets.y,cal_accel_offets.z);
    DBGLN("MPU6050 Gyro Offs:  x=%d,y=%d,z=%d",cal_gyro_offsets.x, cal_gyro_offsets.y,cal_gyro_offsets.z);

    setupOrientation();
    //DBGLN("MPU6050: Gyro Calibration");
    //mpu->CalibrateGyro(8); // Calibrate Gyro only that can change with temp
    //DBGLN("MPU6050 Gyro New Offs:  x=%d,y=%d,z=%d",mpu->getXGyroOffset(), mpu->getYGyroOffset(), mpu->getZGyroOffset());

    DBGLN("MPU6050: Ready");
}

bool MPUDev_MPU6050::rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) 
{
    uint8_t buffer[14];

    //do the same, but retun error:  mpu->getMotion6(ax, ay, az, gx, gy, gz);
    bool readOk = I2Cdev::readBytes(m_address, MPU6050_RA_ACCEL_XOUT_H, 14, buffer, 1) == 14;

    if (readOk) {
        *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
        *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
        *az = (((int16_t)buffer[4]) << 8) | buffer[5];
        *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
        *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
        *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
    } else {
        *ax = *ay = *az = 0;
        *gx = *gy = *gz = 0;
    } 

    return readOk;
}

#endif
