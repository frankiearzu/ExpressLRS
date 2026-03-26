#include "targets.h"

#if defined(HAS_GYRO)
#include "gyro.h"
#include "mixer.h"
#include "logging.h"
#include "elrs_eeprom.h" // only needed to satisfy PIO
#include "config.h"

#ifdef GYRO_DEVICE_MPU6050
#include "mpu/mpu_mpu6050.h"
#endif

#ifdef GYRO_DEVICE_LSM6DXX
#include "mpu/mpu_lsm6dXX.h"
#endif

extern boolean i2c_enabled;
extern boolean spi_enabled;

Gyro gyro = Gyro();
static MPU_Base *mpuDev;

static bool initialize()
{
    mpuDev = nullptr;

    // I2C Gyros
    if (i2c_enabled)
    {
#ifdef GYRO_DEVICE_MPU6050
        if (mpuDev == nullptr)
        {
            mpuDev = new MPUDev_MPU6050();
            if (mpuDev->initialize()) {
                DBGLN("devGyro.init(): Detected MPU6050 Gyro");
            } else {
                mpuDev = nullptr;
            } 
        }
#endif
    }

    // SPI Gyros
    if(spi_enabled) {
#ifdef GYRO_DEVICE_LSM6DXX 
        if (mpuDev == nullptr)
        {
            mpuDev = new MPUDev_LSM6DXX();
            if (mpuDev->initialize()) {
                DBGLN("devGyro.init(): Detected LSM6DXX Gyro");
            } else {
                mpuDev=nullptr;
            }
        }
#endif
    } // if SPI

    if (mpuDev==nullptr) {
         DBGLN("devGyro.init(): Gyro Not Detected");
    }

    // Call Init even when mpuDev is null to disable other parts looking 
    // if gyro is OFF
    gyro.init(mpuDev);

    return mpuDev!=nullptr;
}

static bool gyro_detect() {
    return mpuDev != nullptr;
}

static int start()
{
    DBGLN("devGyro.start()");
    if (!mixer_initialize() || !gyro_detect()) {
        DBGLN("Gyro initialization failed");
        return DURATION_NEVER;
    }
    gyro.start();
    return DURATION_IMMEDIATELY; // Call timeout() immediately;
}

static int timeout()
{
    return gyro.tick();
}

static int event()
{
    DBGLN("deGyro.Event()");
    gyro.event();
    return DURATION_IGNORE;
}

device_t Gyro_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
    .subscribe = EVENT_CONFIG_GYRO_CHANGED,
};

#endif