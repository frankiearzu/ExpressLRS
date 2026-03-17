#include "targets.h"

#if defined(HAS_GYRO) && defined(GYRO_DEVICE_LSM6DXX)
#include "logging.h"
#include "config.h"
#include "mpu_lsm6dXX.h"
#include "I2Cdev.h"
#include "mpu_lsm6dxx_regs.h"

#define LSM6DSV16X_ADDRESS_LOW  0x6A     // Default
#define LSM6DSV16X_ADDRESS_HIGH 0x6B

#define I2C_MASTER_FREQ_HZ 400000

static uint8_t accScaleCode, gyroScaleCode;

static uint8_t lsm6dID = 0x6C;

static void lsm6dxxConfig(MPU_Base *mpu)
{ 
    // Reset the device (wait 100ms before continuing config)
    mpu->writeRegisterBits(LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C_RESET, BIT(0));
    delay(100);

    // Configure data ready pulsed mode
    mpu->writeRegisterBits(LSM6DXX_REG_COUNTER_BDR1, LSM6DXX_MASK_COUNTER_BDR1, 
        LSM6DXX_VAL_COUNTER_BDR1_DDRY_PM);
    
    // Configure interrupt pin 1 for gyro data ready only
    mpu->writeRegister(LSM6DXX_REG_INT1_CTRL, LSM6DXX_VAL_INT1_CTRL_ENABLE);

    // Disable interrupt pin 2
    mpu->writeRegister(LSM6DXX_REG_INT2_CTRL, LSM6DXX_VAL_INT2_CTRL_DISABLE);

    // Configure the accelerometer
    mpu->writeRegister(LSM6DXX_REG_CTRL1_XL, 
        (LSM6DXX_VAL_CTRL1_XL_ODR833 << 4) |// ODR 833Hz
        (LSM6DXX_VAL_CTRL1_XL_2G << 2) |    // 2G Scale
        (LSM6DXX_VAL_CTRL1_XL_LPF2 << 1));  // LPF2 output (default with ODR/4 cutoff)

    // Configure the gyro
    mpu->writeRegister(LSM6DXX_REG_CTRL2_G, 
        (LSM6DXX_VAL_CTRL2_G_ODR6664 << 4) |  // 6664hz ODR
        (LSM6DXX_VAL_CTRL2_G_2000DPS << 2));  // 2000dps scale

    // Configure control register 3
    mpu->writeRegisterBits(LSM6DXX_REG_CTRL3_C, LSM6DXX_MASK_CTRL3_C, 
        (LSM6DXX_VAL_CTRL3_C_H_LACTIVE | // latch LSB/MSB during reads;
         LSM6DXX_VAL_CTRL3_C_PP_OD |     // set interrupt pins active high; set interrupt pins push/pull;
         LSM6DXX_VAL_CTRL3_C_SIM |       // set 4-wire SPI; 
         LSM6DXX_VAL_CTRL3_C_IF_INC));   // enable auto-increment burst reads

    // Configure control register 4
    mpu->writeRegisterBits(LSM6DXX_REG_CTRL4_C, LSM6DXX_MASK_CTRL4_C, 
        (LSM6DXX_VAL_CTRL4_C_DRDY_ENABLED | // enable accelerometer high performane mode;
         LSM6DXX_VAL_CTRL4_C_SPI_DISABLE |  // disable SPI
         LSM6DXX_VAL_CTRL4_C_LPF1_SEL_G));  // enable gyro LPF1

    // Configure control register 6
    mpu->writeRegisterBits(LSM6DXX_REG_CTRL6_C, (lsm6dID == LSM6DSO_CHIP_ID? LSM6DXX_MASK_CTRL6_C:LSM6DSL_MASK_CTRL6_C), 
        (LSM6DXX_VAL_CTRL6_C_XL_HM_MODE | 
         LSM6DXX_VAL_CTRL6_C_FTYPE_300HZ   // set gyro LPF1 cutoff 335.5Hz
        ));

    // Configure control register 7
    mpu->writeRegisterBits(LSM6DXX_REG_CTRL7_G, LSM6DXX_MASK_CTRL7_G, 
        (LSM6DXX_VAL_CTRL7_G_HP_EN_G |    // enable gyro high-pass filter
         LSM6DXX_VAL_CTRL7_G_HPM_G_16));  // gyro HPF cutoff 16mHz

    // Configure control register 9
    if(lsm6dID == LSM6DSO_CHIP_ID) {
        mpu->writeRegisterBits(LSM6DXX_REG_CTRL9_XL, LSM6DXX_MASK_CTRL9_XL, 
            LSM6DXX_VAL_CTRL9_XL_I3C_DISABLE);  // disable I3C interface
    }
}

static bool lsm6dxxDetect(MPU_Base *mpu)
{
    uint8_t tmp = 0;
    uint8_t attemptsRemaining = 5;
    //busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    do {
        delay(150);

        mpu->readRegister(LSM6DXX_REG_WHO_AM_I, &tmp,  1);

        switch (tmp) {
            case LSM6DSO_CHIP_ID:
            case LSM6DSL_CHIP_ID: 
                 lsm6dID = tmp;
                // Compatible chip detected
                return true;
            default:
                // Retry detection
                break;
        }
    } while (attemptsRemaining--);

    return false;
}

static bool lsm6dxxAccGyroRead(MPU_Base *mpu, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t data[12];
    const bool ack = mpu->readRegister(LSM6DXX_REG_OUTX_L_G, data, 12);
    if (!ack) {
        return false;
    }
    *gx = data[0] | static_cast<uint16_t>(data[1] << 8);
    *gy = data[2] | static_cast<uint16_t>(data[3] << 8);
    *gz = data[4] | static_cast<uint16_t>(data[5] << 8);

    *ax = data[6] | static_cast<uint16_t>(data[7] << 8);
    *ay = data[8] | static_cast<uint16_t>(data[9] << 8);
    *az = data[10] | static_cast<uint16_t>(data[11] << 8);
    return true;
}


bool MPUDev_LSM6DXX::initialize() {
    MPU_Base::initialize();
    m_address = LSM6DSV16X_ADDRESS_HIGH;
    
    Wire.setClock(I2C_MASTER_FREQ_HZ);

    // Test The connection 
    DBGLN("Detecting LSM6DXX");
    if (!lsm6dxxDetect(this)) {
        DBGLN("LSM6DXX not found!!");
        return false;
    }

    DBGLN("LSM6DXX found!!");

    accScaleCode = LSM6DXX_VAL_CTRL1_XL_2G; // Acceleation 2G
    accScale1G =   16384.0;  

    gyroScaleCode = LSM6DXX_VAL_CTRL2_G_2000DPS;  
    gyroScaleRad = 2000.0 / 32768.0 / 180 * PI;  //   multiply adc by this to get rad°/s
    gyroScaleDeg = 2000.0 / 32768.0 / 100;       //   multiply adc by this to get deg°/s     
    
    //1.0f / 16.4f; // 2000 dps

    orientationIsWrong = true;
    return true;
}

void MPUDev_LSM6DXX::start() {
    DBGLN("LSM6DXX Start");
    
    lsm6dxxConfig(this);

    memcpy(&cal_accel_offets,config.GetAccelCalibration(),sizeof(rx_config_gyro_calibration_t));
    memcpy(&cal_gyro_offsets,config.GetGyroCalibration(),sizeof(rx_config_gyro_calibration_t));

    DBGLN("LSM6DXX Acc Offs:  x=%d,y=%d,z=%d",cal_accel_offets.x, cal_accel_offets.y,cal_accel_offets.z);
    DBGLN("LSM6DXX Gyro Offs:  x=%d,y=%d,z=%d",cal_gyro_offsets.x, cal_gyro_offsets.y,cal_gyro_offsets.z);

    setupOrientation();
    //DBGLN("LSM6DXX: Gyro Calibration");
    //mpu->CalibrateGyro(8); // Calibrate Gyro only that can change with temp
    //DBGLN("LSM6DXX Gyro New Offs:  x=%d,y=%d,z=%d",mpu->getXGyroOffset(), mpu->getYGyroOffset(), mpu->getZGyroOffset());

    DBGLN("LSM6DXX: Ready");
}


bool MPUDev_LSM6DXX::read(float accel_rpy[], float angle_rpy[]) {
    if (orientationIsWrong) return false;

    rawRead(&v_accel.x, &v_accel.y, &v_accel.z,
            &v_gyro.x,  &v_gyro.y,  &v_gyro.z); 

    v_accel.x -= cal_accel_offets.x;
    v_accel.y -= cal_accel_offets.y;
    v_accel.z -= cal_accel_offets.z;
    
    v_gyro.x -= cal_gyro_offsets.x;
    v_gyro.y -= cal_gyro_offsets.y;
    v_gyro.z -= cal_gyro_offsets.z;

    applyOrientation(&v_accel);
    applyOrientation(&v_gyro);
    
    float gx = ((float) v_gyro.x) * gyroScaleRad;
    float gy = ((float) v_gyro.y) * gyroScaleRad;
    float gz = ((float) v_gyro.z) * gyroScaleRad;

    // use Mahoney filter
    static long last = micros(); // Behaves like Global
    long now = micros();
    float deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    
    Mahony_update(v_accel.x, v_accel.y, v_accel.z, 
                    gx, gy, gz, 
                    deltat, &q);

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

bool MPUDev_LSM6DXX::rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) 
{
    return lsm6dxxAccGyroRead(this, ax, ay, az, gx, gy, gz);
}

#endif
