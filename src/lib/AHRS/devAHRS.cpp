#include <math.h>
#include <Arduino.h>
#include <Wire.h>

#include "devAHRS.h"

#if defined(HAS_AHRS)
#include "config.h"
#include "logging.h"

#include "IMUBase.h"
#include "ICMSeries.h"
#include "MPU6050.h"
#include "QMI8658C.h"
#include "Fusion.h"


static  IMUBase *imu = nullptr;
static  AHRS *ahrs = nullptr;


static void initialize()
{
    // Initializing the IMU
    imu = new MPU6050();
    if (imu->initialize()) {
        DBGLN("Found MPU6050 IMU")
    }
    else {
        delete imu;
        imu = new QMI8658C();
        if (imu->initialize()) {
            DBGLN("Found QMI8658C IMU")
        }
        else {
            delete imu;
            imu = new ICMSeries();
            if (imu->initialize()) {
                DBGLN("Found ICM Series IMU")
            }
            else {
                delete imu;
                imu = nullptr;
                return;
            }
        }
    }

    imu->setInterruptHandler(PIN_INT);

    ahrs = new AHRS();
    ahrs -> initialize(imu);
}

static int start()
{
    if (imu == nullptr)
    {
        return DURATION_NEVER;
    }

    ahrs->start();
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    if (ahrs->calibrating || !ahrs->isRunning()) {
        return 1000; // Try back in 1sec        
    }
    
    ahrs->updateAHRS();

    return DURATION_IMMEDIATELY;
}

device_t AHRS_device = {
    .initialize = initialize,
    .start = start,
    .event = nullptr,
    .timeout = timeout
};

#endif
