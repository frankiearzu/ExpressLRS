#include "targets.h"

// Comment to use old Algorithm openXsensor/UNI-RX algorithm instead of Fusin AHRS
//#define USE_FUSION_AHRS

#if defined(HAS_GYRO)
#include "mpu.h"
#include "logging.h"
#include "config.h"
#include "mpu_biasFilter.h"
#include <Wire.h>

#define MAX_GYRO_DIFF 200       
#define MAX_ACC_DIFF  500 

#if defined (USE_FUSION_AHRS)
#include "Fusion.h"

static FusionAhrs ahrs;

#endif

// Generic
//const char* mpuOrientationNames[8] = {
//    "FRONT(X+)", "BACK(X-)", "LEFT(Y+)", "RIGHT(Y-)", "UP(Z+)", "DOWN(Z-)", "WRONG", "WRONG"};


char **mpuOrientationNames;

// HR8EG
static const char *gyroRxOrientationsHR[8] = 
    {"UART Up(X+)","UART Dn(X-)","Pins Up(Y+)","Pins Dn(Y-)","Lbl Up(Z+)","Lbl Dn(Z-)","WRONG","WRONG"};
// RM
static const char *gyroRxOrientationsRM[8] = 
    {"Pins Up(X+)","Pins Dn(X-)","V-Lbl Up(Y+)","V-Lbl Dn(Y-)","Lbl Up(Z+)","Lbl Dn(Z-)","WRONG","WRONG"};


static int8_t orientationList[36][6] = {
{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,1,1}, {1,2,0,-1,-1,1}, {2,1,0,1,-1,1}, {2,1,0,-1,1,1},\

{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,-1,-1}, {1,2,0,-1,1,-1}, {2,1,0,1,1,-1}, {2,1,0,-1,-1,-1},\

{0,2,1,1,-1,1}, {0,2,1,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,1,1}, {2,0,1,-1,-1,1},\

{0,2,1,1,1,-1}, {0,2,1,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,-1,-1}, {2,0,1,-1,1,-1},\

{0,1,2,1,1,1}, {0,1,2,-1,-1,1}, {1,0,2,1,-1,1}, {1,0,2,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0},\

{0,1,2,1,-1,-1}, {0,1,2,-1,1,-1}, {1,0,2,1,1,-1}, {1,0,2,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}};


bool MPU_Base::initialize() {
    orientationIsWrong = true;
    //imgGyroCalNeeded = true;
    memset(&cal_gyro_offsets,0,sizeof(cal_gyro_offsets));
    memset(&cal_accel_offets,0,sizeof(cal_accel_offets));
    read_errors = 0;

    mpuOrientationNames = (char **) (OPT_HAS_GYRO_MPU6050?gyroRxOrientationsHR:gyroRxOrientationsRM);

    gyroBiasInitialise(gyroSampleRate);

#if defined (USE_FUSION_AHRS)    
    #define GYRO_RANGE  (gyroScaleDeg * 32768.0)
    
    FusionAhrsInitialise(&ahrs);
    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = GYRO_RANGE, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 2U * gyroSampleRate, /* 2 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
#else
   
#endif


    return false;
}

bool MPU_Base::readRegister(uint8_t reg, uint8_t *data, size_t size)
{
    size_t r = 0;
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    if (Wire.endTransmission() == 0)
    {
        Wire.requestFrom(m_address, size);
        r = Wire.readBytes(data, size);
    }
    return r == size;
}

void MPU_Base::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t data = value;
    Wire.beginTransmission(m_address);
    Wire.write(reg);
    Wire.write(&data, 1);
    Wire.endTransmission();
    //return (Wire.endTransmission() == 0);
}

void MPU_Base::writeRegisterBits(uint8_t registerID, uint8_t mask, uint8_t value)
{
    uint8_t newValue;

    if (readRegister(registerID, &newValue, 1)) {
        delayMicroseconds(2);
        newValue = (newValue & ~mask) | value;
        writeRegister(registerID, newValue);
    }
}

void MPU_Base::start() {
    read_errors = 0; // Reset Errors
}

uint8_t MPU_Base::event() {
    return DURATION_IGNORE;
}

bool MPU_Base::isRunning() {
    return !orientationIsWrong;
}

/**
 * This method is used instead of mpu->dmpGetYawPitchRoll() as that method has
 * issues when gravity switches at high pitch angles.
*/
void MPU_Base::GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity)
{
    /* using Gravity Vector */
    
    // yaw: (about Z axis)
    data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);    
    // pitch: (nose up/down, about Y axis)
    data[1] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2(gravity -> y , gravity -> z);
    

    /* using Quartilion */
    /*
    // yaw: (about Z axis)
    data[0] = -atan2((q->x * q->y + q->w * q->z), 0.5 - (q->y * q->y + q->z * q->z));
    // pitch: (nose up/down, about Y axis)
    data[1] = -asin(2.0 * (q->w * q->y - q->x * q->z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan2((q->w * q->x + q->y * q->z), 0.5 - (q->x * q->x + q->y * q->y));
    */


    // NOTE: This is buggy at high pitch angles when gravity flips
    // if (gravity -> z < 0) {
    //     if(data[1] > 0) {
    //         data[1] = PI - data[1];
    //     } else {
    //         data[1] = -PI - data[1];
    //     }
    // }
}

uint8_t MPU_Base::GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

void MPU_Base::applyOrientation(VectorInt16 *v)
{
    // take care of the orientation of the sensor in the model
    float t[3];
    t[0] = v->x;
    t[1] = v->y;
    t[2] = v->z;

    v->x = t[orientationX] * orientationSignX;
    v->y = t[orientationY] * orientationSignY;
    v->z = t[orientationZ] * orientationSignZ;
}

bool MPU_Base::read(float accel_rpy[], float angle_rpy[]) {
    if (orientationIsWrong) return false;

    // Regular READ
    if (!rawRead(&v_accel.x, &v_accel.y, &v_accel.z, 
            &v_gyro.x,  &v_gyro.y,  &v_gyro.z)) {
        read_errors++;
        return false;
    }

    v_accel.x -= cal_accel_offets.x;
    v_accel.y -= cal_accel_offets.y;
    v_accel.z -= cal_accel_offets.z;
    
    v_gyro.x -= cal_gyro_offsets.x;
    v_gyro.y -= cal_gyro_offsets.y;
    v_gyro.z -= cal_gyro_offsets.z;

    applyOrientation(&v_accel);
    applyOrientation(&v_gyro);
    
    // use Mahoney filter
    static long last = micros(); // Behaves like Global
    long now = micros();
    float deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    
    // Get Gyro in Degrees/sec
    VectorFloat gDeg;
    gDeg.x = ((float) v_gyro.x) * gyroScaleDeg;
    gDeg.y = ((float) v_gyro.y) * gyroScaleDeg;
    gDeg.z = ((float) v_gyro.z) * gyroScaleDeg;

    gyroBiasUpdate(gDeg);

    #if defined(USE_FUSION_AHRS)
        FusionVector g, a;
        // Accel in Gs
        a.axis.x = v_accel.x * accScaleG;
        a.axis.y = v_accel.y * accScaleG;
        a.axis.z = v_accel.z * accScaleG;

        // Gyro in Deg/sec
        g.axis.x = gDeg.x;
        g.axis.y = gDeg.y;
        g.axis.z = gDeg.z;

        FusionAhrsUpdate(&ahrs, g, a, FUSION_VECTOR_ZERO, deltat);
        FusionQuaternion qq = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(qq);

        // Accel in  rad/s
        accel_rpy[0] = radians(g.axis.x);  // Roll
        accel_rpy[1] = radians(g.axis.y);  // Pitch
        accel_rpy[2] = radians(g.axis.z);  // Yaw

        // In Rad
        angle_rpy[0] = radians(euler.angle.roll);   // Roll
        angle_rpy[1] = - radians(euler.angle.pitch);  // Pitch
        angle_rpy[2] = radians(euler.angle.yaw);    // Yaw 
        
        // Move to our Quarterion and Gravity and ypr
        q.w = qq.element.w;
        q.x = qq.element.x;
        q.y = qq.element.y;
        q.z = qq.element.z;

        GetGravity(&gravity, &q);

        ypr[0] = angle_rpy[2];
        ypr[1] = angle_rpy[1];
        ypr[2] = angle_rpy[0];
    #else
        Mahony_update(v_accel.x, v_accel.y, v_accel.z, 
                        radians(gDeg.x), radians(gDeg.y), radians(gDeg.z),
                        deltat, &q);

        GetGravity(&gravity, &q);
        GetYawPitchRoll(ypr, &q, &gravity);

        // Accel in  rad/s
        accel_rpy[0] = radians(gDeg.x); // Roll
        accel_rpy[1] = radians(gDeg.y); // Pitch
        accel_rpy[2] = radians(gDeg.z); // Yaw

        // In Rad
        angle_rpy[0] = ypr[2];  // Roll
        angle_rpy[1] = ypr[1];  // Pitch
        angle_rpy[2] = ypr[0];  // Yaw
    #endif

    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats(now);
    #endif

    // unsigned long time_since_update = micros() - last_gyro_update;
    last_gyro_update = last;
    return true;
}


void MPU_Base::setupOrientation()
{
    uint8_t idx;
    orientationIsWrong = false;
    mpuOrientationH = config.GetGyroOrientationH();
    mpuOrientationV = config.GetGyroOrientationV();

    if (mpuOrientationH>5 || mpuOrientationV>5 ) 
    {
        orientationIsWrong = true;
        DBGLN("Orientation is WRONG");
        return;
    }
    idx = mpuOrientationH *6 + mpuOrientationV;  // into a number in range 0/35
    if (orientationList[idx][3] == 0){   // check that combination H and V is valid
        orientationIsWrong = true;
        return;
    }
    orientationX = orientationList[idx][0]; // orientation list contains e.g. 3,2,1,-1,1,1 (first are the index to map, last 3 the sign)
    orientationY = orientationList[idx][1];
    orientationZ = orientationList[idx][2];
    orientationSignX = orientationList[idx][3];
    orientationSignY = orientationList[idx][4];
    orientationSignZ = orientationList[idx][5];

    DBGLN("OrientationH: %s", mpuOrientationNames[mpuOrientationH]);
    DBGLN("OrientationV: %s", mpuOrientationNames[mpuOrientationV]);
}

void MPU_Base::findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx ){
    // find the index and sign of gravity 
    //      idx:  0=X, 1=Y, 2=Z; 
    //      sign: 1=gravity is the opposite (normally Z axis is up and give 1) 
    
    float oneG_70percent = acc1G_adc*0.7;

    if ((float) ax > oneG_70percent) { idx = 0 ;
    } else if ((float) ax < -oneG_70percent) { idx = 1 ;
    } else if ((float) ay >  oneG_70percent) { idx = 2 ;
    } else if ((float) ay < -oneG_70percent) { idx = 3 ;
    } else if ((float) az >  oneG_70percent) { idx = 4 ;
    } else if ((float) az < -oneG_70percent) { idx = 5 ;
    } else { idx= 6; };
    
    DBGLN("findGravty(): ax=%d  ay=%d  az=%d  yawIdx=%d  scale=%f", ax , ay ,  az, idx, acc1G_adc);
}    

uint8_t MPU_Base::readAndGetGravity(){ // return index of orientation; return 6 in case of error
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
    uint8_t idx = 6; 
    
    uint8_t i = 3;
    do {
        if (rawRead(&ax, &ay, &az, &gx, &gy, &gz)) {
            findGravity( ax , ay , az , idx);
            break;
        }
        i =- 1;
    } while (i > 0);
	return idx;
}

void MPU_Base::OrientationHorizontalExecute()  // 
{
    orientationIsWrong = true;
    mpuOrientationH = 6;
    mpuOrientationV = 6;

    DBGLN("Horizontal Detection...");
    uint8_t idx = readAndGetGravity(); // // read the Acc and detect which face is on the upper side 
    if (idx > 5){
         DBGLN("Error during horizontal orientation: direction of gravity has not been found");
         return;
    }
    DBGLN("Upper face of MPU (when model is horizontal) is %s", mpuOrientationNames[idx]);
    mpuOrientationH =  idx ; // save the orientationH 
    
    // Run the calibration, but not saving the offsets
    CalibrateAccel(8, &cal_accel_offets);
    CalibrateGyro(8, &cal_gyro_offsets);
}

void MPU_Base::OrientationVerticalExecute() {
    mpuOrientationV = 6;
    DBGLN("Vertical Detection...");
    uint8_t idx = readAndGetGravity(); // // read the Acc and detect which face is on the upper side 
    if (idx > 5){
        DBGLN("Error during vertical orientation: direction of gravity has not been found");
    }
    DBGLN("Upper face (with nose up) is %s",mpuOrientationNames[idx]);
    mpuOrientationV =  idx ; // save the orientationV

    config.SetGyroOrientation(mpuOrientationH,mpuOrientationV);  

    // Save the Calibration
    config.SetAccelCalibration(cal_accel_offets.x, cal_accel_offets.y, cal_accel_offets.z);
    config.SetGyroCalibration(cal_gyro_offsets.x, cal_gyro_offsets.y, cal_gyro_offsets.z);

    // Restart with the saved config Offsets
    start();
}


//--------------------------------------------------------------------------------------------------
// Mahony scheme uses proportional and integral filtering on
// the error between estimated reference vector (gravity) and measured one.
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// last update 07/09/2020 SJR minor edits
//--------------------------------------------------------------------------------------------------
// IMU algorithm update

#define KP1_LOW_LIMIT 0.975
#define KP1_HIGH_LIMIT 1.025
#define KP1 2.0
#define KI1 0.0

#define KP2_LOW_LIMIT 0.950
#define KP2_HIGH_LIMIT 1.050
#define KP2 1.0
#define KI2 0.0

// currently ax, ay, az are in raw values (+- 32768) and gx,gy,gz are in rad/sec.
void MPU_Base::Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat, Quaternion *q) 
{
#if not defined(USE_FUSION_AHRS)
     
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  //error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms    
    float kp;
    float ki;

    float tmp = (ax * ax) + (ay * ay) + (az * az);
    float totalAccRaw = sqrt(tmp);

    float totalAccG = totalAccRaw / acc1G_adc ; // convert in 1g to perform the comparison and to select best kp and ki
    
    if (( totalAccG > KP1_LOW_LIMIT ) and ( totalAccG < KP1_HIGH_LIMIT )) { //When total acceleration is within some limits (close 1g)
        kp = KP1;
        ki = KI1;
    } else if (( totalAccG > KP2_LOW_LIMIT ) and ( totalAccG < KP2_HIGH_LIMIT )) { //When total acceleration is within some limits (close 1g)
        kp = KP2;
        ki = KI2;
    } else {
        kp = 0;
        ki = 0;
    }


    // 
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    
    if (tmp > 0.0) {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / totalAccRaw;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q->x * q->z - q->w * q->y;
        vy = q->w * q->x + q->y * q->z;
        vz = q->w * q->w - 0.5f + q->z * q->z;

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (ki > 0.0f) {
            ix += ki * ex * deltat;  // integral error scaled by Ki
            iy += ki * ey * deltat;
            iz += ki * ez * deltat;
            gx += ix;  // apply integral feedback
            gy += iy;
            gz += iz;
        }
        
        if (kp > 0.0 ) {
            // Apply proportional feedback to gyro term
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        } else { // total gravity is out of limits, discard accelerations and reset I terms
            ix=0;
            iy=0;
            iz=0;
        } 
    }

    // Integrate rate of change of quaternion, q cross gyro term
    deltat = 0.5 * deltat;
    gx *= deltat;   // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = q->w;
    qb = q->x;
    qc = q->y;
    q->w += (-qb * gx - qc * gy - q->z * gz);
    q->x += (qa * gx + qc * gz - q->z * gy);
    q->y += (qa * gy - qb * gz + q->z * gx);
    q->z += (qa * gz + qb * gy - qc * gx);

    // renormalise quaternion
    tmp = (q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z);
    if ( tmp == 0 ) return;

    recipNorm = 1.0 / sqrt(tmp);
    q->w = q->w * recipNorm;
    q->x = q->x * recipNorm;
    q->y = q->y * recipNorm;
    q->z = q->z * recipNorm;
#endif 
}

bool MPU_Base::AutoCalibrateGyro(int32_t gx, int32_t gy, int32_t gz) 
{
    #define NUMBER_ITER_CALIB 1000
    static int countSkip = 0;
    static int count = 0;
    static int32_t gxAccum = 0, gyAccum = 0, gzAccum = 0; 
    static int32_t gxMin = 60000 , gxMax = -60000 , gyMin = 60000 , gyMax = -60000 , gzMin = 60000 , gzMax = -60000;

    if (countSkip < NUMBER_ITER_CALIB) { // About 1s delay
        countSkip++;
        return false;
    }

    if (count < NUMBER_ITER_CALIB)
    {
        count++;
        
        gxAccum += (int32_t) gx;
        gyAccum += (int32_t) gy;
        gzAccum += (int32_t) gz;

        if (gx < gxMin) gxMin = gx;
        if (gy < gyMin) gyMin = gy;
        if (gz < gzMin) gzMin = gz;
        if (gx > gxMax) gxMax = gx;
        if (gy > gyMax) gyMax = gy;
        if (gz > gzMax) gzMax = gz;
    } else {
        //imgGyroCalNeeded = false; // avoid calibration (it is done)
        DBGLN ("Auto Gyro Calibration: gyro differences: x=%d (%d/%d) y=%d (%d/%d) z=%d (%d/%d)", 
                 gxMax - gxMin, gxMax, gxMin,
                 gxMax - gxMin, gxMax, gxMin,
                 gzMax - gzMin, gzMax, gzMin);

        if ( ((gxMax-gxMin) > MAX_GYRO_DIFF) or ((gyMax-gyMin) > MAX_GYRO_DIFF) or ((gzMax-gzMin) > MAX_GYRO_DIFF) ){
            DBGLN("Auto Gyro calibration failed; will uses gyro offsets saved during horizontal calibration");
            DBGLN("Auto Gyro: Using Offset x=%d  y=%d   z=%d\n",  cal_gyro_offsets.x ,cal_gyro_offsets.y,cal_gyro_offsets.z);
        } else {
            int32_t new_x = gxAccum / NUMBER_ITER_CALIB;
            int32_t new_y = gyAccum / NUMBER_ITER_CALIB;
            int32_t new_z = gzAccum / NUMBER_ITER_CALIB;

            if (cal_gyro_offsets.x != new_x || cal_gyro_offsets.y != new_y || cal_gyro_offsets.z != new_z) {
                DBGLN("Auto Gyro: Succeded, changes detected");
                    
                DBGLN("Auto Gyro: Old Offset x=%d  y=%d   z=%d",  cal_gyro_offsets.x ,cal_gyro_offsets.y,cal_gyro_offsets.z);
                cal_gyro_offsets.x = new_x;
                cal_gyro_offsets.y = new_y;
                cal_gyro_offsets.z = new_z;
                DBGLN("Auto Gyro: New Offset x=%d  y=%d   z=%d",  cal_gyro_offsets.x ,cal_gyro_offsets.y,cal_gyro_offsets.z);

                //config.SetGyroCalibration(
                //    cal_gyro_offsets.x,
                //    cal_gyro_offsets.y,
                //    cal_gyro_offsets.z
                //);
                //config.Commit();
            } else {
                DBGLN("Auto Gyro: Succeded, no change in calibration");
            }
            
        }
    }    
    return true;
}

bool MPU_Base::CalibrateGyro(int8_t loops, rx_config_gyro_calibration_t *offsets)
{
   #define ACCEL_NUM_AVG_SAMPLES	300

    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;

    int32_t gxAccum, gyAccum, gzAccum , gxMin , gxMax , gyMin , gyMax , gzMin , gzMax;
	gxAccum = gyAccum = gzAccum = 0;
    gxMin = gyMin = gzMin = 60000;
    gxMax = gyMax = gzMax = -60000;  

    int16_t errors = 0;

    DBGLN ("Stating Gyro Calibration..");
    calibrating = true;

	for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
        DBG(".");
        int c=0;
        while (!isDataReady() && c++ < 20) { delayMicroseconds(50); }
        if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
            errors++;
            continue;
        }

        gxAccum += (int32_t) gx;
        gyAccum += (int32_t) gy;
        gzAccum += (int32_t) gz;

        if (gx < gxMin) gxMin = gx;
        if (gy < gyMin) gyMin = gy;
        if (gz < gzMin) gzMin = gz;
        if (gx > gxMax) gxMax = gx;
        if (gy > gyMax) gyMax = gy;
        if (gz > gzMax) gzMax = gz;
    }

    DBGLN ("\nGyro Calibration completed..");

    DBGLN ("Calibration: gyro differences: x=%d (%d/%d) y=%d (%d/%d) z=%d (%d/%d)", 
                 gxMax - gxMin, gxMax, gxMin,
                 gxMax - gxMin, gxMax, gxMin,
                 gzMax - gzMin, gzMax, gzMin);

    
    if (((gxMax - gxMin) > MAX_GYRO_DIFF) or ((gyMax - gyMin) > MAX_GYRO_DIFF) or ((gzMax - gzMin) > MAX_GYRO_DIFF)) {
        DBGLN ("Error in IMU calibration: to much variations in the gyro values");
        calibrating = false;
        return false;
    }

    if (errors > 50) {
        DBGLN("Too many read errors during calibration  (Errors=%d)",errors);
        calibrating = false;
        return false;
    }

    gxAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);
	gyAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);
	gzAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);
   
    offsets->x = (int16_t)(gxAccum);
	offsets->y = (int16_t)(gyAccum);
	offsets->z = (int16_t)(gzAccum);
 
    DBGLN("Gyr Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);
    calibrating = false;
    return true;
}

bool MPU_Base::CalibrateAccel(int8_t loops, rx_config_gyro_calibration_t *offsets)
{
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
	int32_t axAccum, ayAccum, azAccum , axMin , axMax , ayMin , ayMax , azMin , azMax;
    int16_t errors = 0;
   
    axAccum = ayAccum = azAccum = 0;
    axMin = ayMin = azMin  = 60000;
    axMax = ayMax = azMax  = -60000;  
    
    DBGLN ("Stating Accelerometer Calibration. OrientationH:%s",mpuOrientationNames[mpuOrientationH]);
    calibrating = true;

    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){
        DBG(".");      
        int c=0;
        while (!isDataReady() && c++ < 20) { delayMicroseconds(50); }
        if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
            errors++;
            continue;
        }
        

        // Remove Gravity
        switch (mpuOrientationH) {
            case 0:  ax -= acc1G_adc; break;
            case 1:  ax += acc1G_adc; break;
            case 2:  ay -= acc1G_adc; break;
            case 3:  ay += acc1G_adc; break;
            case 4:  az -= acc1G_adc; break;
            case 5:  az += acc1G_adc; break;
        }
       
        axAccum += (int32_t) ax;
        ayAccum += (int32_t) ay;
        azAccum += (int32_t) az;

        if (ax < axMin) axMin = ax;
        if (ay < ayMin) ayMin = ay;
        if (az < azMin) azMin = az;
        if (ax > axMax) axMax = ax;
        if (ay > ayMax) ayMax = ay;
        if (az > azMax) azMax = az;        
    }

    DBGLN ("\nAccelerometer Calibration completed..");

    DBGLN ("Calibration: acceleration differences: x=%d (%d/%d) y=%d (%d/%d) z=%d (%d/%d)", 
                 axMax - axMin, axMax, axMin,
                 axMax - axMin, axMax, axMin,
                 azMax - azMin, azMax, azMin);


    // here we know the Acc but still will reject the measurement if noise is to big
    
    if (((axMax - axMin) > MAX_ACC_DIFF) or ((ayMax - ayMin) > MAX_ACC_DIFF) or ((azMax - azMin) > MAX_ACC_DIFF)) {
        DBGLN ("Error in IMU calibration: to much variations in the acceleration values: x=%d y=%d z=%d", axMax - axMin, ayMax - ayMin , azMax - azMin);
        calibrating = false;
        return false;
    }

    if (errors > 50) {
        DBGLN("Too many read errors during calibration  (Errors=%d)",errors);
        calibrating = false;
        return false;
    }

    axAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);
    ayAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);
    azAccum /= (ACCEL_NUM_AVG_SAMPLES - errors);

    // we store the values
    offsets->x = (int16_t)(axAccum);
	offsets->y = (int16_t)(ayAccum);
	offsets->z = (int16_t)(azAccum);

    DBGLN("Acc Offs:  x=%d,y=%d,z=%d",offsets->x, offsets->y,offsets->z);
    calibrating = false;
    return true;
}

void MPU_Base::calibrate(bool save)
{
    // Run the calibration
    CalibrateAccel(8, &cal_accel_offets);
    CalibrateGyro(8, &cal_gyro_offsets);

    if (!save) return;

    config.SetAccelCalibration(
        cal_accel_offets.x,
        cal_accel_offets.y,
        cal_accel_offets.z
    );
    config.SetGyroCalibration(
        cal_gyro_offsets.x,
        cal_gyro_offsets.y,
        cal_gyro_offsets.z
    );
}


#ifdef DEBUG_GYRO_STATS
/**
 * For debugging print useful gyro state
 */
void MPU_Base::print_gyro_stats(long nowMicros)
{
    static long last_gyro_stats_time = 0;
    static int update_rate = 0;

    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int current_rate = 1.0 / ((nowMicros - last_gyro_update) / 1000000.0);
    update_rate = (update_rate + current_rate) / 2;  // Average

    char rate_str[15]; sprintf(rate_str, "%4d", update_rate);

    char pitch_str[15]; sprintf(pitch_str, "%6.2f", degrees(ypr[1]));
    char roll_str[15]; sprintf(roll_str, "%6.2f", degrees(ypr[2]));
    char yaw_str[15]; sprintf(yaw_str, "%6.2f", degrees(ypr[0]));

    char gyro_x[15]; sprintf(gyro_x, "%6.3f", (double) v_gyro.x * gyroScaleDeg);
    char gyro_y[15]; sprintf(gyro_y, "%6.3f", (double) v_gyro.y * gyroScaleDeg);
    char gyro_z[15]; sprintf(gyro_z, "%6.3f", (double) v_gyro.z * gyroScaleDeg);

    char accel_x[15]; sprintf(accel_x, "%6.3f", (double) v_accel.x * accScaleG);
    char accel_y[15]; sprintf(accel_y, "%6.3f", (double) v_accel.y * accScaleG);
    char accel_z[15]; sprintf(accel_z, "%6.3f", (double) v_accel.z * accScaleG);

    char gravity_x[15]; sprintf(gravity_x, "%4.3f", gravity.x);
    char gravity_y[15]; sprintf(gravity_y, "%4.3f", gravity.y);
    char gravity_z[15]; sprintf(gravity_z, "%4.3f", gravity.z);


    VectorFloat  o = getBiasOffsets();
    char bias_x[15]; sprintf(bias_x, "%4.3f", o.x);
    char bias_y[15]; sprintf(bias_y, "%4.3f", o.y);
    char bias_z[15]; sprintf(bias_z, "%4.3f", o.z);

    // Uncomment lines needed for debugging
    DBGLN("Refresh: %s HZ ",rate_str);
    DBGLN("Pitch:%s Roll:%s Yaw:%s",pitch_str, roll_str, yaw_str);
    DBGLN("Q       (w: %f, x: %f, y: %f, z: %f)",q.w, q.x, q.y, q.z);
    DBGLN("Gyro    (x: %s, y: %s, z: %s)",gyro_x, gyro_y, gyro_z);
    DBGLN("GBias   (x: %s, y: %s, z: %s)",bias_x, bias_y, bias_z);
    DBGLN("Accel   (x: %s, y: %s, z: %s)",accel_x, accel_y, accel_z);
    DBGLN("Gravity (x: %s, y: %s, z: %s)",gravity_x, gravity_y, gravity_z);

    last_gyro_stats_time = millis();
}
#endif

uint8_t MPU_Base::m_address = 0;

#endif
