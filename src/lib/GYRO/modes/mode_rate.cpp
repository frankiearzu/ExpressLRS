#if defined(HAS_GYRO)

#include "gyro.h"
#include "mixer.h"
#include "mode_rate.h"
#include "pid.h"
#include "logging.h"

/**
 * Airplane Normal Mode / Rate Mode
 *
 * This is a basic "wind rejection mode" and counteracts roll and pitch changes.
 *
 * As the channel command increases the correction decreases allowing unlimited
 * angular rates.
 */

void _make_gyro_debug_string(PID *pid, char *str) {
    sprintf(str, "Setpoint: %5.2f PV: %5.2f I:%5.2f D:%5.2f Error: %5.2f Out: %5.2f",
        pid->setpoint, pid->pv, pid->Iout, pid->Dout, pid->error, pid->output);

}

 RateController::RateController()
 {
 }

void RateController::configure_pid_gains(PID *pid, const rx_config_gyro_PID_t *pid_params, int8_t gain,
                         float max, float min)
{
    DBG("Config gains: [P=%d I=%d D=%d G=%d] ", pid_params->p, pid_params->i, pid_params->d, (int8_t) gain);
    if (max == 0.0 && min == 0.0) {
        // not gyro correction on this axis
        pid->configure(0.0, 0.0, 0.0, 0.0, 0.0);
    } else {
        float p = gain * pid_params->p / 1000.0;
        float i = gain * pid_params->i / 1000.0;
        float d = gain * pid_params->d / 1000.0;
        DBG("PID: [P=%f I=%f D=%f]", p, i, d);

        pid->configure(p, i, d, max, min);
    }
    DBGLN("");
    pid->reset();
}

void  RateController::applyFModeSettings(gyro_mode_t fm) {
    fm_settings.raw =  config.GetGyroFMode(GYRO_MODE_OFF)->raw; // Default settings for ALL

    auto rate_settings = config.GetGyroFMode(fm); // Get settings for Specific flight Mode

    if (rate_settings->val.angleMaxEnable) { // Override
        fm_settings.val.angleMaxPitch = rate_settings->val.angleMaxPitch;
        fm_settings.val.angleMaxRoll = rate_settings->val.angleMaxRoll;
    } else {
         fm_settings.val.angleMaxPitch=0;
         fm_settings.val.angleMaxRoll=0;
    }

    if (rate_settings->val.trimEnable) { // Add to Defaults, that is Level Flight
        fm_settings.val.trimPitch += (int8_t) rate_settings->val.trimPitch;
        fm_settings.val.trimRoll += (int8_t) rate_settings->val.trimRoll;
    }

    if (fm_settings.val.angleMaxPitch+fm_settings.val.angleMaxRoll > 0) {
        DBGLN("Angle Limits: Pitch=%d Roll=%d",(int8_t)fm_settings.val.angleMaxPitch, (int8_t)fm_settings.val.angleMaxRoll);
    }

    if (fm_settings.val.trimPitch+fm_settings.val.trimRoll != 0) {
       DBGLN("Trims: Pitch=%d Poll=%d",(int8_t)fm_settings.val.trimPitch, (int8_t)fm_settings.val.trimRoll);
    }
}


bool RateController::isInverted(float angle_rpy[]) {
    float angleDeg = fabs(angle_rpy[GYRO_AXIS_ROLL]);
    return angleDeg > 100;
}

bool RateController::isHighPitch(float angle_rpy[]) {
    float angleDeg = fabs(angle_rpy[GYRO_AXIS_PITCH]);
    return angleDeg > 80;
}

void RateController::initialize(gyro_mode_t mode)
{
    RateController::mode = mode;

    float roll_limit = 1.0;
    float pitch_limit = 1.0;
    float yaw_limit = 1.0;

    applyFModeSettings(mode);

    const rx_config_gyro_PID_t *roll_pid_params     = config.GetGyroPID(GYRO_AXIS_ROLL);
    const rx_config_gyro_PID_t *pitch_pid_params    = config.GetGyroPID(GYRO_AXIS_PITCH);
    const rx_config_gyro_PID_t *yaw_pid_params      = config.GetGyroPID(GYRO_AXIS_YAW);

    configure_pid_gains(&pid_roll,  roll_pid_params,    fm_settings.val.gainRoll,   roll_limit, -1.0 * roll_limit);
    configure_pid_gains(&pid_pitch, pitch_pid_params,   fm_settings.val.gainPitch,  pitch_limit, -1.0 * pitch_limit);
    configure_pid_gains(&pid_yaw,   yaw_pid_params,     fm_settings.val.gainYaw,    yaw_limit, -1.0 * yaw_limit);

    ignore_input[0] = ignore_input[1] = ignore_input[2] = false;
}

void RateController::calculate_stick_pri(float input_rpt[]) {
    // Modulate the correction depending on how much axis stick command
    //float stick_gain_div = (1 << stick_priority); // 1,2,4
    //float percent = (1/stick_gain_div)-fabs(roll_in);
    //if (percent < 0) percent = 0.0;

    // Modulate the correction depending on how much axis stick command
    for (int8_t axis = 0; axis < 3; axis++) {
        stick_pri[axis]  = 1 - fabs(input_rpy[axis]);
    }
}

void RateController::calculate_pid(float input_rpy[], float acc_rpy[], float ang_rpy[])
{
    ignore_input[0] = ignore_input[1] = ignore_input[2] = false;
    int8_t  rateMult = 4;   // 25% of full-Scale Accelerometer rate (TODO: Make it configurable)

    // Copy parameters to internal class variables
    for (int8_t axis = 0; axis < 3; axis++) {
        RateController::input_rpy[axis] = input_rpy[axis];
    }

    calculate_stick_pri(input_rpy);

    // Desired angular rate is zero
    pid_roll.calculate(0, acc_rpy[GYRO_AXIS_ROLL]  / rateMult);
    pid_pitch.calculate(0, acc_rpy[GYRO_AXIS_PITCH] / rateMult);
    pid_yaw.calculate(0, -(acc_rpy[GYRO_AXIS_YAW]/ rateMult));

    corr[GYRO_AXIS_ROLL]  = pid_roll.output  * stick_pri[GYRO_AXIS_ROLL]  * gyro.master_gain;
    corr[GYRO_AXIS_PITCH] = pid_pitch.output * stick_pri[GYRO_AXIS_PITCH]  * gyro.master_gain;
    corr[GYRO_AXIS_YAW]   = pid_yaw.output   * stick_pri[GYRO_AXIS_YAW]    * gyro.master_gain;
}

void RateController::printState() {
        char piddebug[128];
        DBGLN("MASTER GAIN %f", gyro.master_gain);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", radToDeg(gyro.angle_rpy[0]), radToDeg(gyro.angle_rpy[1]), radToDeg(gyro.angle_rpy[2])); 
        DBGLN("Angles:  %s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", input_rpy[GYRO_AXIS_ROLL], input_rpy[GYRO_AXIS_PITCH], input_rpy[GYRO_AXIS_YAW]);    
        DBGLN("Cmds:    %s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f", stick_pri[GYRO_AXIS_ROLL], stick_pri[GYRO_AXIS_PITCH], stick_pri[GYRO_AXIS_YAW]);
        DBGLN("StickPri:%s",piddebug);
        sprintf(piddebug,"Roll:%5.2f Pitch:%5.2f Yaw:%5.2f",corr[GYRO_AXIS_ROLL], corr[GYRO_AXIS_PITCH], corr[GYRO_AXIS_YAW]);
        DBGLN("Corr    :%s",piddebug);

        _make_gyro_debug_string(&pid_pitch, piddebug);
        DBGLN("PID Pitch %s", piddebug);
        _make_gyro_debug_string(&pid_roll, piddebug);
        DBGLN("PID Roll  %s", piddebug);
        _make_gyro_debug_string(&pid_yaw, piddebug);
        DBGLN("PID Yaw   %s", piddebug);
}

uint16_t RateController::applyCorrection(uint8_t ch, gyro_output_channel_function_t channel_function, float command, bool inverted) {
    float correction = 0.0;
    if (channel_function==FN_AILERON) {
        correction = (inverted)?-corr[GYRO_AXIS_ROLL]:corr[GYRO_AXIS_ROLL];
        if (ignore_input[GYRO_AXIS_ROLL]) {
            command = 0;
        }
    } else if (channel_function==FN_ELEVATOR) {
        correction = (inverted)?-corr[GYRO_AXIS_PITCH]:corr[GYRO_AXIS_PITCH];
        if (ignore_input[GYRO_AXIS_PITCH]) {
            command = 0;
        }
    } 
    else if (channel_function==FN_ELEVON_L || channel_function==FN_ELEVON_R) {
        correction = (inverted)?-corr[GYRO_AXIS_PITCH]:corr[GYRO_AXIS_PITCH]; // Invert elevator if needed
        auto cor2 = (channel_function==FN_ELEVON_L)?corr[GYRO_AXIS_ROLL]:-corr[GYRO_AXIS_ROLL]; // Invert Aileron depending of Left/Right 
        correction = (correction + cor2)/2;
        if (ignore_input[GYRO_AXIS_PITCH]) {
            command = 0;
        }
    }
    else if (channel_function==FN_VTAIL_L || channel_function==FN_VTAIL_R) {
        correction = (inverted)?-corr[GYRO_AXIS_PITCH]:corr[GYRO_AXIS_PITCH]; // Invert elevator if needed
        auto cor2 = (channel_function==FN_VTAIL_L)?corr[GYRO_AXIS_YAW]:-corr[GYRO_AXIS_YAW]; // Invert Rud depending of Left/Right 
        correction = (correction + cor2)/2;
        if (ignore_input[GYRO_AXIS_PITCH]) {
            command = 0;
        }
    }
    else if (channel_function==FN_RUDDER) {
        correction = (inverted)?-corr[GYRO_AXIS_YAW]:corr[GYRO_AXIS_YAW]; // Invert rudder if needed
    }

    return float_to_us(ch, command + correction);
}

#endif
