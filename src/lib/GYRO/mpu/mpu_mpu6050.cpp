#include "targets.h"

#if defined(HAS_GYRO) && defined(GYRO_DEVICE_MPU6050)
#include "mpu_mpu6050.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "logging.h"
#include "config.h"

 // 0: Use Mahony Algorithm to compute Quarterions
 // 1: USE Hardware Dynamic Motion Processor to compute Quarterions
 //         NOTE: When using DMP, it does not seem when flyinf with Auto-Level or Envelope/SAFE
 //               When doing a full roll stick movement, it bank to the max angle, then slowly continue increasing the
 //               bank.. Looks like the motor vibration or something is affecting it.. it does not happen with Mahoney
#define USE_DMP  0  

#define I2C_MASTER_FREQ_HZ 400000

#if USE_DMP
#define MPU6050_INT_DPM             (1<<MPU6050_INTERRUPT_DMP_INT_BIT)    // 0x02
#define MPU6050_INT_FIFO_OFLOW      (1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT) // 0x10

static uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
//static uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
static uint16_t fifoCount;       // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64];   // FIFO storage buffer
#endif

static uint8_t accScaleCode, gyroScaleCode;


const char * MPUDev_MPU6050::GetMPUName() {
    return "MPU6050";
}

bool MPUDev_MPU6050::initialize() {
    MPU_Base::initialize();
    m_address = MPU6050_DEFAULT_ADDRESS;

    DBGLN("Detecting MPU6050");
    //Wire.setClock(I2C_MASTER_FREQ_HZ);
    mpu =  new MPU6050();

    bool found = false;
    for (int8_t i=0;i<5;i++) {
        if (mpu->testConnection()) 
        {
            found = true;
            break;
        }
        vTaskDelay(50 * portTICK_PERIOD_MS);
    }

    if (!found) 
    {
        DBGLN("MPU6050 not found!");
        mpu = nullptr;
        return false;
    }

    accScaleCode = MPU6050_ACCEL_FS_2; // Acceleation 2G
    accScale1G = 16384.0;              // +/-  32768/2 

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

    #if USE_DMP
        mpu->dmpInitialize();
    #else
        mpu->initialize();
    #endif

    mpu->setFullScaleAccelRange(accScaleCode);
    mpu->setFullScaleGyroRange(gyroScaleCode);
    mpu->setMasterClockSpeed(MPU6050_CLOCK_DIV_400); // 400kHz that matches Wire Clock
    
    const rx_config_gyro_calibration_t *offsets;
    offsets = config.GetAccelCalibration();
    mpu->setXAccelOffset(offsets->x);
    mpu->setYAccelOffset(offsets->y);
    mpu->setZAccelOffset(offsets->z);
    DBGLN("MPU6050 Acc Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);

    offsets = config.GetGyroCalibration();
    mpu->setXGyroOffset(offsets->x);
    mpu->setYGyroOffset(offsets->y);
    mpu->setZGyroOffset(offsets->z);
    DBGLN("MPU6050 Gyro Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);

    vTaskDelay(50 * portTICK_PERIOD_MS);

    setupOrientation();
    //DBGLN("MPU6050: Gyro Calibration");
    //mpu->CalibrateGyro(8); // Calibrate Gyro only that can change with temp
    //DBGLN("MPU6050 Gyro New Offs:  x=%d,y=%d,z=%d",mpu->getXGyroOffset(), mpu->getYGyroOffset(), mpu->getZGyroOffset());

    #if USE_DMP
        mpu->setDMPEnabled(true);
    #endif
    DBGLN("MPU6050: Ready");
}


bool MPUDev_MPU6050::read(float accel_rpy[], float angle_rpy[]) {
    if (orientationIsWrong) return false;

#if USE_DMP
    mpuIntStatus = mpu->getIntStatus();
    fifoCount = mpu->getFIFOCount();
    if ((mpuIntStatus & MPU6050_INT_FIFO_OFLOW) || fifoCount == 1024) 
    {
        DBGLN("Resetting gyro FIFO buffer");
        mpu->resetFIFO();
        return false;
    }
    else if ((mpuIntStatus & MPU6050_INT_DPM) == 0) // 0x02
    {
        return false;
    }
    
    int result = mpu->GetCurrentFIFOPacket(fifoBuffer, 28);
    if (result != 1)
        return DURATION_IMMEDIATELY;

    mpu->dmpGetGyro(&v_gyro, fifoBuffer);
    mpu->dmpGetAccel(&v_accel, fifoBuffer);
    mpu->dmpGetQuaternion(&q, fifoBuffer); // [w, x, y, z] quaternion container
#else
    // Regular READ
    if (!rawRead(&v_accel.x, &v_accel.y, &v_accel.z, 
            &v_gyro.x,  &v_gyro.y,  &v_gyro.z)) {
        return false;
    }
#endif

    applyOrientation(&v_gyro);
    
    float gx = ((float) v_gyro.x) * gyroScaleRad;
    float gy = ((float) v_gyro.y) * gyroScaleRad;
    float gz = ((float) v_gyro.z) * gyroScaleRad;

#if USE_DMP
    applyOrientation(&q);
#else  
    applyOrientation(&v_accel);

    // use Mahoney filter
    static long last = micros(); // Behaves like Global
    long now = micros();
    float deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    
    Mahony_update(v_accel.x, v_accel.y, v_accel.z, 
                    gx, gy, gz, 
                    deltat, &q);
#endif
    //mpu->dmpGetEuler(euler, &q);
    GetGravity(&gravity, &q);
    GetYawPitchRoll(ypr, &q, &gravity);

    // Conver from rad/s -> Degres/s ???

    accel_rpy[0] = gx; // Roll
    accel_rpy[1] = gy; // Pitch
    accel_rpy[2] = gz; // Yaw
    
    angle_rpy[0] = ypr[2];  // Roll
    angle_rpy[1] = ypr[1];  // Pitch
    angle_rpy[2] = ypr[0];  // Yaw
   
    
    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

    // unsigned long time_since_update = micros() - last_gyro_update;
    last_gyro_update = micros();
    return true;
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

bool MPUDev_MPU6050::CalibrateGyro(int8_t loops, rx_config_gyro_calibration_t *offsets)
{   
    // Clear old offsets
    mpu->setXGyroOffset(0);
    mpu->setYGyroOffset(0);
    mpu->setZGyroOffset(0);
    delay(50);

    //MPU_Base::CalibrateGyro(loops,offsets);
    //mpu->setXGyroOffset(0);
    //mpu->setYGyroOffset(0);
    //mpu->setZGyroOffset(0);
    //return true;

    DBGLN ("Stating Gyro Calibration..");
    mpu->CalibrateGyro(8);
    
    // Get the offsets
    offsets->x = mpu->getXGyroOffset();
    offsets->y = mpu->getYGyroOffset();
    offsets->z = mpu->getZGyroOffset();

    DBGLN ("\nGyro Calibration completed..");
    DBGLN("MPU605 Gyr Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);

    // Clear the offsets
    mpu->setXGyroOffset(0);
    mpu->setYGyroOffset(0);
    mpu->setZGyroOffset(0);
    delay(50);

    return true;
}

bool MPUDev_MPU6050::CalibrateAccel(int8_t loops, rx_config_gyro_calibration_t *offsets) {
    mpu->setXAccelOffset(0);
    mpu->setYAccelOffset(0);
    mpu->setZAccelOffset(0);
    delay(50);

    //MPU_Base::CalibrateAccel(loops,offsets);
    //mpu->setXAccelOffset(0);
    //mpu->setYAccelOffset(0);
    //mpu->setZAccelOffset(0);
    //if (true) return true;

    DBGLN ("Stating Accelerometer Calibration..");
    mpu->CalibrateAccel(8);

    offsets->x = mpu->getXAccelOffset();
    offsets->y = mpu->getYAccelOffset();
    offsets->z = mpu->getZAccelOffset();

    DBGLN ("\nAccelerometer Calibration completed..");
    DBGLN("MPU605 Accel Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);

    mpu->setXAccelOffset(0);
    mpu->setYAccelOffset(0);
    mpu->setZAccelOffset(0);
    delay(50);

    return true;
}


#endif
