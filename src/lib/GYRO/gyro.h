#pragma once
#include "targets.h"

#if defined(HAS_GYRO)
#include "device.h"
#include "config.h"
#include "pid.h"
#include "gyro_types.h"
#include <math.h>
#include "helper_3dmath.h"
#include "mpu/mpu.h"
#include "modes/mode.h"

#define GYRO_CODE_VERSION   2

#define GYRO_US_MIN 988
#define GYRO_US_MID 1500
#define GYRO_US_MAX 2012

#define radToDeg(angleInRadians) ((angleInRadians) * RAD_TO_DEG)
#define degToRad(angleInDegrees) ((angleInDegrees) * DEG_TO_RAD)

/**
 * Add some servo jitter feedback to the pilot after the gyro has initialized.
 */
#define GYRO_BOOT_JITTER
#ifdef GYRO_BOOT_JITTER
#define GYRO_BOOT_JITTER_US 45
#define GYRO_BOOT_JITTER_MS 175
#define GYRO_BOOT_JITTER_TIMES 4
#endif

//class MPU_Base;
//class Mode_Base;

class Gyro
{
public:
    void init(MPU_Base *mpu);
    void start();

    gyro_status_t getStatus();
    gyro_mode_t getMode(void);
    void mixerInput();
    void mixerOutput(uint8_t ch, uint16_t *us);
    void send_telemetry();
    //bool read_device();
    int tick();
    uint8_t event();
    void calibrate();
    void reload();
    void pause();
    void StickCenterCalibration();
    void StickLimitCalibration(bool done);

    MPU_Base *mpuDev = nullptr;

    float master_gain = 1.0;
     gyro_mode_t gyro_mode;
// protected:

    // orientation/motion vars
    
    float acc_rpy[3];     // [roll, pitch, yaw] accelearion
    float angle_rpy[3];   // [roll, pitch, yaw] angles

    uint16_t update_rate;
    unsigned long last_update;
    bool initialized;
    gyro_learn_state_t learn_state = GYRO_LEARN_OFF;

private:
   
    Mode_Base* mode_controller;

    int8_t  mode_ch   = -1;
    int8_t  gain_ch   = -1;
    int8_t  roll_ch   = -1;
    int8_t  pitch_ch  = -1;
    int8_t  yaw_ch    = -1;

    void detect_gain(uint16_t us);
    void detect_mode(uint16_t us);
    void switch_mode(gyro_mode_t mode);
    void learn_sticks(uint8_t ch, uint16_t us);

    unsigned long pid_delay=0;
};

extern Gyro gyro;

// configure PID controllers from LUA gains for each axis with the specified limit
// (typically 1.0). Set a limit to 0.0 to disable PID control on an axis.
void configure_pids(float roll_limit, float pitch_limit, float yaw_limit, const rx_config_gyro_fmode_t *fm);

// Helper method to configure a PID controller instance use the rx config values
void configure_pid_gains(PID* pid, const rx_config_gyro_PID_t* pid_params, int8_t gain, float max, float min);
#endif