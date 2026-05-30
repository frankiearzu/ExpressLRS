#include "targets.h"

#if defined(HAS_GYRO)
#include "AHRS.h"
#include "logging.h"
#include "config.h"

/*
    AHRS: Attitude and Heading Reference System

    It uses Madgwick's implementation of Mayhony's AHRS algorithm.
    https://ahrs.readthedocs.io/en/latest/filters/mahony.html

    Also provides a lot of configuration functions to detect RX Orientation, Calibration, etc.
    
*/

// HR8EG
const char *mpuOrientationNames[8] = 
    {"QRC Dn(X+)","QRC Up(X-)","Pins Up(Y+)","Pins Dn(Y-)","Lbl Up(Z+)","Lbl Dn(Z-)","WRONG","WRONG"};


static int8_t orientationList[36][6] = {
{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,1,1}, {1,2,0,-1,-1,1}, {2,1,0,1,-1,1}, {2,1,0,-1,1,1},\

{3,3,3,0,0,0}, {3,3,3,0,0,0}, {1,2,0,1,-1,-1}, {1,2,0,-1,1,-1}, {2,1,0,1,1,-1}, {2,1,0,-1,-1,-1},\

{0,2,1,1,-1,1}, {0,2,1,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,1,1}, {2,0,1,-1,-1,1},\

{0,2,1,1,1,-1}, {0,2,1,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}, {2,0,1,1,-1,-1}, {2,0,1,-1,1,-1},\

{0,1,2,1,1,1}, {0,1,2,-1,-1,1}, {1,0,2,1,-1,1}, {1,0,2,-1,1,1}, {3,3,3,0,0,0}, {3,3,3,0,0,0},\

{0,1,2,1,-1,-1}, {0,1,2,-1,1,-1}, {1,0,2,1,1,-1}, {1,0,2,-1,-1,-1}, {3,3,3,0,0,0}, {3,3,3,0,0,0}};


bool AHRS::initialize(IMUBase * imu) {
    orientationIsWrong = true;

    memset(&cal_gyro_offsets,0,sizeof(cal_gyro_offsets));
    memset(&cal_accel_offets,0,sizeof(cal_accel_offets));
    
    accel_rpy[0] = 0; accel_rpy[1] = 0; accel_rpy[2] = 0;
    angle_rpy[0] = 0; angle_rpy[1] = 0; angle_rpy[2] = 0;

    this->imu = imu;

    if (imu != nullptr) {
        accScale1G   = imu->getAccScale1G();
        gyroScaleRad = imu->getGyroScaleRad();
    }

    return true;
}


void AHRS::start() {
    read_errors = 0; // Reset Errors

    memcpy(&cal_gyro_offsets,config.GetGyroCalibration(),sizeof(cal_gyro_offsets));
    memcpy(&cal_accel_offets,config.GetAccelCalibration(),sizeof(cal_accel_offets));

    imu->start();

    setupOrientation();
}

uint8_t AHRS::event() {
    return DURATION_IGNORE;
}

bool AHRS::isRunning() {
    return !orientationIsWrong && imu != nullptr;
}

void AHRS::getAccel_RPY(float rpy[]) {
    memcpy(rpy,accel_rpy,sizeof(accel_rpy));
}

void AHRS::getAngle_RPY(float rpy[]) {
    memcpy(rpy,angle_rpy,sizeof(angle_rpy));
}

/**
 * This method is used instead of mpu->dmpGetYawPitchRoll() as that method has
 * issues when gravity switches at high pitch angles.
*/
void AHRS::GetRollPitchYaw(float *data, Quaternion *q, VectorFloat *gravity)
{
    // yaw: (about Z axis)
    data[GYRO_AXIS_YAW] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[GYRO_AXIS_PITCH] = atan2(gravity -> x , sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[GYRO_AXIS_ROLL] = atan2(gravity -> y , gravity -> z);

    // NOTE: This is buggy at high pitch angles when gravity flips
    // if (gravity -> z < 0) {
    //     if(data[GYRO_AXIS_PITCH] > 0) {
    //         data[GYRO_AXIS_PITCH] = PI - data[GYRO_AXIS_PITCH];
    //     } else {
    //         data[GYRO_AXIS_PITCH] = -PI - data[GYRO_AXIS_PITCH];
    //     }
    // }
}

uint8_t AHRS::GetGravity(VectorFloat *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
    return 0;
}

void AHRS::applyOrientation(VectorInt16 *v)
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

/*
    AHRS: Attitude and Heading Reference System update

    It uses Madgwick's implementation of Mayhony's AHRS algorithm.
    https://ahrs.readthedocs.io/en/latest/filters/mahony.html
    
*/
bool AHRS::updateAHRS() {
    int16_t ax,ay,az, gx, gy, gz;

    if (!imu->isDataReady()) return false;
    imu->rawRead(&ax,&ay,&az,&gx,&gy,&gz);

    if (!isRunning()) return false;

    // Apply calibration offsets
    v_accel.x =  ax - cal_accel_offets.x;
    v_accel.y =  ay - cal_accel_offets.y;
    v_accel.z =  az - cal_accel_offets.z;
    
    v_gyro.x = gx - cal_gyro_offsets.x;
    v_gyro.y = gy - cal_gyro_offsets.y;
    v_gyro.z = gz - cal_gyro_offsets.z;

    // Apply IMU/RX orientation
    applyOrientation(&v_accel);
    applyOrientation(&v_gyro);
    
    // Convert raw data to Rad/Sec   
    float gxf = ((float) v_gyro.x) * gyroScaleRad;
    float gyf = ((float) v_gyro.y) * gyroScaleRad;
    float gzf = ((float) v_gyro.z) * gyroScaleRad;

    // Use Mahoney filter
    static long last = micros(); // Behaves like Global
    long now = micros();
    float deltat = ((float)(now - last))* 1.0e-6; //seconds since last update
    last = now;
    
    Mahony_update((float) v_accel.x, (float) v_accel.y, (float) v_accel.z, 
                    gxf, gyf, gzf, 
                    deltat, &q);

    GetGravity(&gravity, &q);
    GetRollPitchYaw(angle_rpy, &q, &gravity);

    accel_rpy[0] = gxf; // Roll
    accel_rpy[1] = gyf; // Pitch
    accel_rpy[2] = gzf; // Yaw
    
    #ifdef DEBUG_GYRO_STATS
    print_gyro_stats();
    #endif

    // unsigned long time_since_update = micros() - last_gyro_update;
    last_gyro_update = micros();
    return true;
}

static boolean isOrientationValid(uint8_t h, uint8_t v) {
    if (h>5 || v>5 ) 
    {
        return false;
    }
    uint8_t idx = h *6 + v;  // into a number in range 0/35
    if (orientationList[idx][3] == 0){   // check that combination H and V is valid
        return false;
    }
    return true;
}

void AHRS::setupOrientation()
{
    uint8_t idx;

    mpuOrientationH = config.GetGyroOrientationH();
    mpuOrientationV = config.GetGyroOrientationV();

    orientationIsWrong = !isOrientationValid(mpuOrientationH,mpuOrientationV);

    if (orientationIsWrong) 
    {
        DBGLN("Orientation is WRONG");
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

void AHRS::findGravity(int32_t ax, int32_t ay, int32_t az, uint8_t &idx ){
    // find the index and sign of gravity 
    //      idx:  0=X, 1=Y, 2=Z; 
    //      sign: 1=gravity is the opposite (normally Z axis is up and give 1) 
    
    float oneG_70percent = accScale1G*0.7;

    if ((float) ax > oneG_70percent) { idx = 0 ;
    } else if ((float) ax < -oneG_70percent) { idx = 1 ;
    } else if ((float) ay >  oneG_70percent) { idx = 2 ;
    } else if ((float) ay < -oneG_70percent) { idx = 3 ;
    } else if ((float) az >  oneG_70percent) { idx = 4 ;
    } else if ((float) az < -oneG_70percent) { idx = 5 ;
    } else { idx= 6; };
    
    DBGLN("findGravty(): ax=%d  ay=%d  az=%d  yawIdx=%d  scale=%f", ax , ay ,  az, idx, accScale1G);
}    

uint8_t AHRS::readAndGetGravity(){ // return index of orientation; return 6 in case of error
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

void AHRS::OrientationHorizontalExecute()  // 
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

void AHRS::OrientationVerticalExecute() {
    mpuOrientationV = 6;
    DBGLN("Vertical Detection...");
    uint8_t idx = readAndGetGravity(); // // read the Acc and detect which face is on the upper side 
    if (idx > 5){
        DBGLN("Error during vertical orientation: direction of gravity has not been found");
    }
    DBGLN("Upper face (with nose up) is %s",mpuOrientationNames[idx]);
    mpuOrientationV =  idx ; // save the orientationV

    orientationIsWrong = !isOrientationValid(mpuOrientationH,mpuOrientationV);

    if (orientationIsWrong) {
        DBGLN("Orientation is WRONG");
        mpuOrientationH = 6;
        mpuOrientationV = 6;
    }

    config.SetGyroOrientation(mpuOrientationH,mpuOrientationV);  

    // Save the Calibration
    config.SetAccelCalibration(cal_accel_offets.x, cal_accel_offets.y, cal_accel_offets.z);
    config.SetGyroCalibration(cal_gyro_offsets.x, cal_gyro_offsets.y, cal_gyro_offsets.z);
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

/*
    currently ax, ay, az are in raw values (+- 32768) and gx,gy,gz are in rad/sec. 
*/
void AHRS::Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat, Quaternion *q) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;  //error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms    
    float kp;
    float ki;

    float tmp = ax * ax + ay * ay + az * az;
    float totalAccRaw = sqrt(tmp);
    float totalAccG = totalAccRaw / accScale1G ; // convert in 1g to perform the comparison and to select best kp and ki
    
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
    tmp = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if ( tmp == 0 ) return;
    recipNorm = 1.0 / sqrt(tmp);
    q->w = q->w * recipNorm;
    q->x = q->x * recipNorm;
    q->y = q->y * recipNorm;
    q->z = q->z * recipNorm;
}

bool AHRS::CalibrateGyro(int8_t loops, rx_config_gyro_calibration_t *offsets)
{
   #define ACCEL_NUM_AVG_SAMPLES	150

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
        delayMicroseconds(1000); // take 8 msec with dlpf = 20 ; 1900us when BW = 188
        if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
            delayMicroseconds(100); // Read Again if error
            if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
                errors++;
                 DBG("*");
                continue;
            }
        }
        DBG(".");

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

    #define MAX_GYRO_DIFF 200
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

bool AHRS::CalibrateAccel(int8_t loops, rx_config_gyro_calibration_t *offsets)
{
    int16_t ax,ay,az ;
    int16_t gx,gy,gz ;
	int32_t axAccum, ayAccum, azAccum , axMin , axMax , ayMin , ayMax , azMin , azMax;
	axAccum = ayAccum = azAccum = 0;
    int16_t errors = 0;
   
    axMin = ayMin = azMin  = 60000;
    axMax = ayMax = azMax  = -60000;  
    
    DBGLN ("Stating Accelerometer Calibration..");
    calibrating = true;

    for (int inx = 0; inx < ACCEL_NUM_AVG_SAMPLES; inx++){     
        delayMicroseconds(1000); // take 8 msec with dlpf = 20 ; 1900us when BW = 188
        if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
            delayMicroseconds(100);  // Read Again if Error
            if (!rawRead(&ax,&ay,&az,&gx,&gy,&gz)) {
                errors++;
                 DBG("*"); 
                continue;
            }
        }
        DBG("."); 

        // Remove Gravity
        switch (mpuOrientationH) {
            case 0:
            case 1:  ax -= accScale1G; break;
            case 2:
            case 3:  ay -= accScale1G; break;
            case 4:
            case 5:  az -= accScale1G; break;
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
    #define MAX_ACC_DIFF 500
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

void AHRS::calibrateLevel(bool save)
{
    if (mpuOrientationH > 5) {
        DBGLN ("Horizontal Orientation Invalid");
        return;
    }

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
void AHRS::print_gyro_stats()
{
    static long last_gyro_stats_time = 0;

    if (millis() - last_gyro_stats_time < 500)
        return;

    // Calculate gyro update rate in HZ
    int update_rate = 1.0 /
                      ((micros() - last_gyro_update) / 1000000.0);

    char rate_str[5]; sprintf(rate_str, "%4d", update_rate);

    char pitch_str[8]; sprintf(pitch_str, "%6.2f", angle_rpy[GYRO_AXIS_PITCH] * 180 / M_PI);
    char roll_str[8]; sprintf(roll_str, "%6.2f", angle_rpy[GYRO_AXIS_ROLL] * 180 / M_PI);
    char yaw_str[8]; sprintf(yaw_str, "%6.2f", angle_rpy[GYRO_AXIS_YAW] * 180 / M_PI);

    char gyro_x[8]; sprintf(gyro_x, "%6.5f", (double) v_gyro.x);
    char gyro_y[8]; sprintf(gyro_y, "%6.5f", (double) v_gyro.y);
    char gyro_z[8]; sprintf(gyro_z, "%6.5f", (double) v_gyro.z);

    char gravity_x[8]; sprintf(gravity_x, "%4.2f", gravity.x);
    char gravity_y[8]; sprintf(gravity_y, "%4.2f", gravity.y);
    char gravity_z[8]; sprintf(gravity_z, "%4.2f", gravity.z);

    char debug_line[128];
    sprintf(debug_line,
        "Raw Accel: (x: %d y: %d z: %d)",
                 v_accel.x, v_accel.y, v_accel.z
    );
    DBGLN(debug_line);

    sprintf(debug_line,
        "G_Scale = %4.2f GyroScale = %2.6f", (double)accScale1G, (double)gyroScaleRad);
    DBGLN(debug_line);

    // Uncomment lines needed for debugging
    DBGLN("Refresh: %s HZ      Read Errors =%d",rate_str, read_errors);
    DBGLN("Pitch:%s Roll:%s Yaw:%s   degrees",pitch_str, roll_str, yaw_str);
    DBGLN("Q       (w: %f, x: %f, y: %f, z: %f)",q.w, q.x, q.y, q.z);
    DBGLN("Gyro    (x: %s, y: %s, z: %s)",gyro_x, gyro_y, gyro_z);
    DBGLN("Gravity (x: %s, y: %s, z: %s)",gravity_x, gravity_y, gravity_z);

    last_gyro_stats_time = millis();
}
#endif

#endif