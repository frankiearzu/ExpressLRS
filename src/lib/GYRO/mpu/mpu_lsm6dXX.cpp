#include "targets.h"

#if defined(HAS_GYRO)
#include "logging.h"
#include "config.h"
#include "mpu_lsm6dXX.h"
#include "I2Cdev.h"
#include "mpu_lsm6dxx_regs.h"


#define USE_SPI     1
#define USE_I2C     0

#define LSM6DSV16X_ADDRESS_LOW  0x6A     // Default
#define LSM6DSV16X_ADDRESS_HIGH 0x6B

static uint8_t accScaleCode, gyroScaleCode;

static uint8_t lsm6dID = 0;

#if USE_I2C

static bool lsm6dxxDetect_I2C(MPU_Base *mpu)
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

static void lsm6dxxConfig_I2C(MPU_Base *mpu)
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

// ==============================
// Check STATUS_REG
// ==============================
static bool lsm6dxxAccGyroDataReady_I2C(MPU_Base *mpu)
{
    uint8_t status;
    mpu->readRegister(LSM6DXX_REG_STATUS, &status, 1);
    // Check Accel (XL) and Gyro data available
    uint8_t bits = LSM6DXX_VAL_STATUS_XLDA | LSM6DXX_VAL_STATUS_GDA; 
    return ((status & bits) == bits); // Has Gyro and Acce;
}


static bool lsm6dxxAccGyroRead_I2C(MPU_Base *mpu, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t data[12];

    /*
    const uint8_t timeout_ms = 5;
    uint32_t start = millis();
    bool dataReady = false;
    while (((millis() - start) < timeout_ms) && ! dataReady) {
        dataReady = lsm6dxxAccGyroDataReady_I2C(mpu);
    }
    if (!dataReady) return false; // No data available
    */

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
#endif  // USE_I2C

#if USE_SPI

#include "SPI.h"
static SPISettings _spiSettings;
extern SPIClass _spi;
#define LSM6D_CS_PIN  10
#define SPI_READ_BIT    0x80

// ==============================
// SPI Read Register
// ==============================
static uint8_t readReg(uint8_t reg)
{
  _spi.beginTransaction(_spiSettings);
  digitalWrite(LSM6D_CS_PIN, LOW);

  _spi.transfer(reg | SPI_READ_BIT);  //Turn on SPI_READ_BIT
  uint8_t val = _spi.transfer(0xFF);

  digitalWrite(LSM6D_CS_PIN, HIGH);
  _spi.endTransaction();
  //DBGLN("SPI DEV LSM6D readReg");
  return val;
}

// ==============================
// SPI Write Register
// ==============================
static void writeReg(uint8_t reg, uint8_t val)
{
  _spi.beginTransaction(_spiSettings);
  digitalWrite(LSM6D_CS_PIN, LOW);

  _spi.transfer(reg);         // 写 = 最高位 0
  _spi.transfer(val);

  digitalWrite(LSM6D_CS_PIN, HIGH);
  _spi.endTransaction();
  DBGLN("SPI DEV LSM6D writeReg");
}

// ==============================
// SPI Burst Read
// ==============================
static void burstRead(uint8_t reg, uint8_t *data, uint8_t length)
{
  _spi.beginTransaction(_spiSettings);
  digitalWrite(LSM6D_CS_PIN, LOW);

  // IF_INC (auto-increment) needs to be setup in CTRL3_C
  _spi.transfer(reg | SPI_READ_BIT);
  for (uint8_t i = 0; i < length; i++) {
    data[i] = _spi.transfer(0xFF);
  }

  digitalWrite(LSM6D_CS_PIN, HIGH);
  _spi.endTransaction();
}

// ==============================
// Check STATUS_REG
// ==============================
static bool lsm6dxxAccGyroDataReady_SPI()
{
    uint8_t status = readReg(LSM6DXX_REG_STATUS);
    // Check Accel (XL) and Gyro data available
    uint8_t bits = LSM6DXX_VAL_STATUS_XLDA | LSM6DXX_VAL_STATUS_GDA; 
    return ((status & bits) == bits); // Has Gyro and Acce;
}

static bool lsm6dxxDetect_SPI(MPU_Base *mpu)
{
    // Read chip ID
    uint8_t id = readReg(LSM6DXX_REG_WHO_AM_I);
    DBGLN("SPI DEV LSM6D ID: 0x%x", id);
    if (id != LSM6DSO_CHIP_ID && id != LSM6DSL_CHIP_ID) {  // 0x6C=LSM6DSO, 0x6A=LSM6DSL
        DBGLN("SPI DEV Unknown chip ID!");
        return false;
    }

    lsm6dID = id;
    return true;
}

static void lsm6dxxConfig_SPI(MPU_Base *mpu) {
    //DBGLN("SPI DEV LSM6D MPUDev_LSM6D_SPI");
    
    // Reset the device
    DBGLN("SPI DEV Resetting...");
    writeReg(LSM6DXX_REG_CTRL3_C, 0x01);  // SW_RESET
    delay(100);

    // Verify reset by reading back
    uint8_t ctrl3 = readReg(LSM6DXX_REG_CTRL3_C);
    DBGLN("SPI DEV CTRL3_C after reset: 0x%x", ctrl3);

    // Configure interrupt pin 1 for gyro data ready only
    writeReg(LSM6DXX_REG_INT1_CTRL, LSM6DXX_VAL_INT1_CTRL_ENABLE);

    // Disable interrupt pin 2
    writeReg(LSM6DXX_REG_INT2_CTRL, LSM6DXX_VAL_INT2_CTRL_DISABLE);

    // Configure accelerometer: 
    uint8_t data = (LSM6DXX_VAL_CTRL1_XL_ODR833 << 4) | // ODR 1.6KHZ,  Original ODR 833Hz
                    (LSM6DXX_VAL_CTRL1_XL_2G << 2) |    // 2G Scale
                    (LSM6DXX_VAL_CTRL1_XL_LPF2 << 1);

    writeReg(LSM6DXX_REG_CTRL1_XL, data);
    DBGLN("SPI DEV CTRL1_XL written: 0x%x", data);

    // Configure gyro: ODR 1.6khz, ±2000dps
    writeReg(LSM6DXX_REG_CTRL2_G,
        (LSM6DXX_VAL_CTRL2_G_ODR833 << 4) |   // ODR 1.6Khz, original 1.6khz ODR
        (LSM6DXX_VAL_CTRL2_G_2000DPS << 2));   // 2000dps scale
    //DBGLN("SPI DEV CTRL2_G written: 0xAC");

    // Configure control register 3 - enable auto-increment for burst reads
    writeReg(LSM6DXX_REG_CTRL3_C, 
                LSM6DXX_VAL_CTRL3_C_IF_INC); // IF_INC enabled
    DBGLN("SPI DEV CTRL3_C configured: 0x04");

    // Configure control register 4
    writeReg(LSM6DXX_REG_CTRL4_C,  
        (LSM6DXX_VAL_CTRL4_C_DRDY_ENABLED | // enable accelerometer high performane mode;
         LSM6DXX_VAL_CTRL4_C_I2C_DISABLE  | // Disable I2C
         LSM6DXX_VAL_CTRL4_C_LPF1_SEL_G));  // enable gyro LPF1

    // Configure control register 6 for Low Pass Filter (LPF1)
    writeReg(LSM6DXX_REG_CTRL6_C,
        (LSM6DXX_VAL_CTRL6_C_XL_HM_MODE |  // High Performance Mode
         LSM6DXX_VAL_CTRL6_C_FTYPE_171HZ   // set gyro LPF1 cutoff 171Hz
        ));

    // NEW: Configure control register 7
    // Set High Pass Filters for Accelerometer
    // Not needed
    //writeReg(LSM6DXX_REG_CTRL7_G, 
    //    (LSM6DXX_VAL_CTRL7_G_HP_EN_G |    // enable gyro high-pass filter
    //     LSM6DXX_VAL_CTRL7_G_HPM_G_16));  // gyro HPF cutoff 16mHz


    // Configure control register 9
    if(lsm6dID == LSM6DSO_CHIP_ID) {
        writeReg(LSM6DXX_REG_CTRL9_XL, 
            LSM6DXX_VAL_CTRL9_XL_I3C_DISABLE);  // disable I3C interface
    }

    // Verify writes
    uint8_t ctrl1 = readReg(LSM6DXX_REG_CTRL1_XL);
    uint8_t ctrl2 = readReg(LSM6DXX_REG_CTRL2_G);
    DBGLN("SPI DEV Verify - CTRL1_XL: 0x%x, CTRL2_G: 0x%x", ctrl1, ctrl2);

    DBGLN("SPI DEV LSM6D init complete");
}

static bool lsm6dxxAccGyroRead_SPI(MPU_Base *mpu, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    uint8_t data[12];
    
    /*
    const uint8_t timeout_ms = 5;
    uint32_t start = millis();
    bool dataReady = false;
    while (((millis() - start) < timeout_ms) && ! dataReady) {
        dataReady = lsm6dxxAccGyroDataReady_SPI();
    }
    if (!dataReady) return false; // No data available
    */

    burstRead(LSM6DXX_REG_OUTX_L_G, data, 12);

    *gx = data[0] | static_cast<uint16_t>(data[1] << 8);
    *gy = data[2] | static_cast<uint16_t>(data[3] << 8);
    *gz = data[4] | static_cast<uint16_t>(data[5] << 8);

    *ax = data[6] | static_cast<uint16_t>(data[7] << 8);
    *ay = data[8] | static_cast<uint16_t>(data[9] << 8);
    *az = data[10] | static_cast<uint16_t>(data[11] << 8);
    return true;
}
#endif


const char * MPUDev_LSM6DXX::GetMPUName() {
    switch (lsm6dID) {
        case LSM6DSO_CHIP_ID: return "LSM6DSO";
        case LSM6DSL_CHIP_ID: return "LSM6DSL";
    } 
    return "LSM6Dxx";
}

bool MPUDev_LSM6DXX::initialize() {
    MPU_Base::initialize();
    m_address = LSM6DSV16X_ADDRESS_HIGH;
    bool found = false;
    
    // Test The connection 
    DBGLN("Detecting LSM6DXX");

#if USE_I2C
    found = lsm6dxxDetect_I2C(this)) {
#endif
#if USE_SPI
    _spiSettings = SPISettings(10000000, MSBFIRST, SPI_MODE3);

    // Initialize CS
    pinMode(LSM6D_CS_PIN, OUTPUT);
    digitalWrite(LSM6D_CS_PIN, HIGH);

    for (int8_t i=0;i<5;i++) {            
        if (lsm6dxxDetect_SPI(this)) {
            found = true;
            break;
        }
        vTaskDelay(50 * portTICK_PERIOD_MS);
    }
#endif

    if (!found) 
    {
        DBGLN("LSM6DXX not found!");
        return false;
    }

    DBGLN("LSM6DXX found!!");

    accScaleCode = LSM6DXX_VAL_CTRL1_XL_2G; // Acceleation 2G
    accScale1G =   32768 / 2;  

    gyroScaleCode = LSM6DXX_VAL_CTRL2_G_2000DPS;  
    gyroScaleRad = 2000.0 / 32768.0 / 180 * PI;  //   multiply adc by this to get rad°/s
    gyroScaleDeg = 2000.0 / 32768.0 / 100;       //   multiply adc by this to get deg°/s     
    
    //1.0f / 16.4f; // 2000 dps

    orientationIsWrong = true;
    return true;
}

void MPUDev_LSM6DXX::start() {
    DBGLN("LSM6DXX Start");
    MPU_Base::start();

#if USE_I2C   
    lsm6dxxConfig_I2C(this);
#endif
#if USE_SPI
    lsm6dxxConfig_SPI(this);
#endif

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

bool MPUDev_LSM6DXX::isDataReady() 
{
#if USE_I2C
    return lsm6dxxAccGyroDataReady_I2C(this);
#endif
#if USE_SPI   
    return lsm6dxxAccGyroDataReady_SPI();
#endif
}

bool MPUDev_LSM6DXX::rawRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) 
{
#if USE_I2C   
    return lsm6dxxAccGyroRead_I2C(this, ax, ay, az, gx, gy, gz);
#endif
#if USE_SPI   
    return lsm6dxxAccGyroRead_SPI(this, ax, ay, az, gx, gy, gz);
#endif
}

#endif // HAS_GYRO
