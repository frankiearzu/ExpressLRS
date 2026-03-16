#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "gyro.h"
#include "gyro_types.h"
#include "mixer.h"
//#include "device.h"
#include "mpu/mpu.h"
#include "modes/mode_safe.h"
#include "modes/mode_level.h"
#include "modes/mode_hover.h"
#include "modes/mode_rate.h"
#include "CRSFRouter.h"
#include "logging.h"


// Comment to Remove Debug of State
#define GYRO_PID_DEBUG_TIME 5000  // Time im Ms

#ifdef GYRO_PID_DEBUG_TIME
unsigned long gyro_debug_time = 0;
#endif

// Channel Data
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];

#define GYRO_SUBTRIM_INIT_SAMPLES 10
static uint8_t stick_subtrim_cycles = 0;
static rx_config_pwm_limits_t temp_limits[PWM_MAX_CHANNELS] = {};

// Must match mixer.h: gyro_input_channel_function_t
static const char* STR_gyroInputChannelMode[] = {"None","Roll","Pitch","Yaw","Mode","Gain"};
// Must match mixer.h: gyro_output_channel_function_t
static const char* STR_gyroOutputChannelMode[] = {"None","Aileron","Elevator","Rudder","Elevon","V Tail"};
// Must match gyro.h gyro_mode_t
static const char* STR_gyroMode[] = {"Off","Rate","SAFE","Level","Launch","Hover"};
// Must match gyro_axis_t
static const char* STR_gyroAxis[] = {"Roll","Pitch","Yaw"}; 

volatile gyro_event_t gyro_event = GYRO_EVENT_NONE;

static Mode_Base*  mode_controllers [6] = { };

#ifdef GYRO_BOOT_JITTER
static uint8_t boot_jitter_times = 0;
static uint32_t boot_jitter_time = 0;
static int8_t boot_jitter_offset = GYRO_BOOT_JITTER_US;

static bool boot_jitter(uint16_t *us)
{
    if (boot_jitter_times > GYRO_BOOT_JITTER_TIMES)
        return false;

    if ((millis() - boot_jitter_time) > GYRO_BOOT_JITTER_MS)
    {
        boot_jitter_times++;
        boot_jitter_time = millis();
        boot_jitter_offset *= -1;
    }

    *us = *us + boot_jitter_offset;
    return true;
}
#endif

/**
 * Return the first channel matching input `mode` or -1 if not found.
*/
static int8_t GetGyroInputChannelNumber(gyro_input_channel_function_t mode)
{
    for (int8_t i = 0; i < GYRO_MAX_CHANNELS; i++) {
        auto info =  config.GetGyroChannel(i);
        if (info->val.input_mode == mode)
            return i;
    }
    return -1;
}

static float channel_us(uint8_t ch)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
    return CRSF_to_US(crsfVal);
}

static float channel_command(uint8_t ch)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
    uint16_t us = CRSF_to_US(crsfVal);
    return us_command_to_float(ch, us);
}

void Gyro::init(MPU_Base *mpu)
{
    DBGLN("Gyro:Init()");
    
    initialized     = false;
    mpuDev          = mpu;
    mode_controller = nullptr;
    gyro_mode       = GYRO_MODE_OFF;
    learn_state     = GYRO_LEARN_OFF;

    mode_controllers[GYRO_MODE_OFF]     = nullptr;
    mode_controllers[GYRO_MODE_RATE]    = new RateController();
    mode_controllers[GYRO_MODE_LEVEL]   = new LevelController();
    mode_controllers[GYRO_MODE_LAUNCH]  = mode_controllers[GYRO_MODE_LEVEL];
    mode_controllers[GYRO_MODE_SAFE]    = new SafeController();
    mode_controllers[GYRO_MODE_HOVER]   = new HoverController();
}

void Gyro::start()
{
    DBGLN("Gyro:Start()");
    initialized = false;
    if (!config.GetGyroEnabled()) return; //not enabled
    if (mpuDev== nullptr) return; // No Gyro Detected
    
    gyro_mode = GYRO_MODE_OFF;
    learn_state = GYRO_LEARN_OFF;
    mpuDev->start();
    initialized = mpuDev->isRunning();

    mode_controller = nullptr;
    mode_ch     = GetGyroInputChannelNumber(FN_IN_GYRO_MODE);
    gain_ch     = GetGyroInputChannelNumber(FN_IN_GYRO_GAIN);
    roll_ch     = GetGyroInputChannelNumber(FN_IN_ROLL);
    pitch_ch    = GetGyroInputChannelNumber(FN_IN_PITCH);
    yaw_ch      = GetGyroInputChannelNumber(FN_IN_YAW);

    if (roll_ch < 0) { // Try to find Elevons
        //roll_ch     = GetGyroInputChannelNumber(FN_IN_ELEVON);
        if (roll_ch >= 0 && pitch_ch < 0) {
            pitch_ch = roll_ch;  // Elevator and Ail shares same channel
        }
    }

    if (yaw_ch < 0) { // Try to find Vtail
        //yaw_ch     = GetGyroInputChannelNumber(FN_IN_VTAIL);
        if (yaw_ch >= 0 && pitch_ch < 0) {
            pitch_ch = yaw_ch;  // Elevator and Rud shares same channel
        }
    }


    #ifdef GYRO_BOOT_JITTER
    boot_jitter_times = 0;
    boot_jitter_time = 0;
    #endif
}

gyro_status_t Gyro::getStatus() 
{
    if (!config.GetGyroEnabled()) return GYRO_STATUS_OFF;
    if (mpuDev== nullptr) return GYRO_STATUS_NOT_DETECTED;
    if (!mpuDev->isRunning()) return GYRO_STATUS_NEED_CALIBRATION; 
    return GYRO_STATUS_OK;
}

void Gyro::calibrate()
{
    initialized = false;
    // Level Calibration
    mpuDev->calibrate(true);
    initialized = mpuDev->isRunning();
}

void Gyro::detect_mode(uint16_t us)
{
    const rx_config_gyro_mode_pos_t *modes = config.GetGyroModePos();
    const uint16_t width = (GYRO_US_MAX - GYRO_US_MIN) / 5;
    uint8_t channel_position = (us - GYRO_US_MIN) / width;
    channel_position = channel_position > 4 ? 4 : channel_position;
    gyro_mode_t selected_mode;
    switch (channel_position)
    {
        case 0: selected_mode = (gyro_mode_t) modes->val.pos1; break;
        case 1: selected_mode = (gyro_mode_t) modes->val.pos2; break;
        case 2: selected_mode = (gyro_mode_t) modes->val.pos3; break;
        case 3: selected_mode = (gyro_mode_t) modes->val.pos4; break;
        case 4: selected_mode = (gyro_mode_t) modes->val.pos5; break;
        default: selected_mode = GYRO_MODE_OFF; break;
    }
    if (gyro_mode != selected_mode)
        switch_mode(selected_mode);
}

/**
 * Trigger a gyro re-initialization of the current gyro mode
*/
void Gyro::reload()
{
    start();
}

/**
 * Trigger a gyro to stop until restarted
*/
void Gyro::pause()
{
    initialized = false;
}


void Gyro::switch_mode(gyro_mode_t mode)
{
    DBGLN("Gyro: Switching mode=[%s]", STR_gyroMode[mode]);
    DBGLN("Gyro: Master Gain=[%f]", master_gain);

    gyro_mode = mode;
    mode_controller = mode_controllers[mode];

    if (mode_controller != nullptr) {
        mode_controller->initialize(mode);
    }
}

void Gyro::detect_gain(uint16_t us)
{
    master_gain = (us_command_to_float(us) + 1) / 2;
    //master_gain = (float(us - GYRO_US_MIN) / (GYRO_US_MAX - GYRO_US_MIN)) * 500;
}



void Gyro::mixerInput()
{
    // We get called before the gyro configuration is initialized
    if (!initialized || learn_state != GYRO_LEARN_OFF) return;

    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    if (mode_ch >= 0) detect_mode(channel_us(mode_ch));
    
    if (mode_controller == nullptr) return;

    float input_rpy[3]  = {0.0, 0.0, 0.0};
   
    if (roll_ch >= 0)   {
        auto info =  config.GetGyroChannel(roll_ch);
        input_rpy[GYRO_AXIS_ROLL]   = channel_command(roll_ch) * ((info->val.inverted)?-1:+1);
    }
    if (pitch_ch >= 0)  {
        auto info =  config.GetGyroChannel(pitch_ch);
        input_rpy[GYRO_AXIS_PITCH]  = channel_command(pitch_ch) * ((info->val.inverted)?-1:+1);
    }

    if (yaw_ch >= 0) {
        auto info =  config.GetGyroChannel(yaw_ch);
        input_rpy[GYRO_AXIS_YAW]    = channel_command(yaw_ch) * ((info->val.inverted)?-1:+1);
    }

    if (gain_ch >= 0)   detect_gain(channel_us(gain_ch)); else master_gain = 1.0;

    mode_controller->calculate_pid(input_rpy, acc_rpy, angle_rpy);

    #ifdef GYRO_PID_DEBUG_TIME
    if (gyro.gyro_mode != GYRO_MODE_OFF &&
        micros() - gyro_debug_time > GYRO_PID_DEBUG_TIME * 1000
    ) {
        mode_controller->printState();
        gyro_debug_time = micros();
    }
    #endif
    
}

/**
 * Apply gyro servo output mixing and detect gyro mode
 */
void Gyro::mixerOutput(uint8_t ch, uint16_t *us)
{
    auto ch_info = config.GetGyroChannel(ch);
    auto output_mode = (gyro_output_channel_function_t) ch_info->val.output_mode;

    // Learning Sticks can happen at any time
    if (learn_state != GYRO_LEARN_OFF) {
        learn_sticks(ch,*us);
        return;
    }

    // We get called before the gyro configuration is initialized
    if (!initialized) return;
   
    if (output_mode == FN_NONE)
        return;

    #ifdef GYRO_BOOT_JITTER
    if (boot_jitter(us))
        return;
    #endif

    if (mode_controller == nullptr) return; // Gyro OFF???

    // Normalize the µs value to a +-1.0 keeping in mind subtrim and max throws
    float command = us_command_to_float(ch, *us);
    *us = mode_controller->applyCorrection(ch, output_mode, command, ch_info->val.inverted);

    // Limit output values to configured limits when is a channel controlled by Gyro
    if (output_mode != FN_NONE) {
        const rx_config_pwm_limits_t *limits = config.GetPwmChannelLimits(ch);
        *us = constrain(*us, limits->val.min, limits->val.max);
    }
}

static int16_t decidegrees2Radians10000(int16_t angle_decidegree)
{
    while (angle_decidegree > 1800)
    {
        angle_decidegree -= 3600;
    }
    while (angle_decidegree < -1800)
    {
        angle_decidegree += 3600;
    }
    return (int16_t)((M_PI / 180.0f) * 1000.0f * angle_decidegree);
}

void Gyro::send_telemetry()
{
    // Get yaw/pitch/roll in decidegrees and convert to uint16_t
    uint16_t rpy16[3] = {0};
    rpy16[GYRO_AXIS_ROLL]   = (uint16_t)(gyro.angle_rpy[GYRO_AXIS_ROLL] * 1800 / M_PI);
    rpy16[GYRO_AXIS_PITCH]  = (uint16_t)(gyro.angle_rpy[GYRO_AXIS_PITCH] * 1800 / M_PI);
    rpy16[GYRO_AXIS_YAW]    = (uint16_t)(gyro.angle_rpy[GYRO_AXIS_YAW] * 1800 / M_PI);
    
    CRSF_MK_FRAME_T(crsf_sensor_attitude_t)
    crsfAttitude = {0};
    crsfAttitude.p.pitch = htobe16(decidegrees2Radians10000(rpy16[GYRO_AXIS_PITCH]));
    crsfAttitude.p.roll = htobe16(decidegrees2Radians10000(rpy16[GYRO_AXIS_ROLL]));
    crsfAttitude.p.yaw = htobe16(decidegrees2Radians10000(rpy16[GYRO_AXIS_YAW]));

    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfAttitude, CRSF_FRAMETYPE_ATTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_attitude_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_RADIO_TRANSMITTER, &crsfAttitude.h);

    CRSF_MK_FRAME_T(crsf_flight_mode_t)
    crsfFlightMode = {0};

    strcpy(crsfFlightMode.p.flight_mode, STR_gyroMode[gyro.gyro_mode]);

    crsfRouter.SetHeaderAndCrc((crsf_header_t *)&crsfFlightMode, CRSF_FRAMETYPE_FLIGHT_MODE, CRSF_FRAME_SIZE(sizeof(crsf_flight_mode_t)));
    crsfRouter.deliverMessageTo(CRSF_ADDRESS_CRSF_TRANSMITTER, &crsfFlightMode.h);
}

int Gyro::tick()
{
    static long tel_delay = 0; // Behaves like Global

    if (!initialized) return 1000; // come back in 1000 ms if not initialized

    if (mpuDev->read(acc_rpy, angle_rpy)) {
        last_update = micros();

        if ((micros() - tel_delay) > 300000 ) { // 300 ms cycle
            tel_delay = micros();
            send_telemetry();
        }
    }

    return 1; //1 ms:  ~1k gyro refresh loop (return for timeout)
    //return DURATION_IMMEDIATELY;
}

uint8_t Gyro::event() 
{
    return mpuDev->event();
}

void Gyro::learn_sticks(uint8_t ch, uint16_t us) {
    if (learn_state== GYRO_LEARN_SUBTRIMS) {
        // Set midpoint (subtrim) from an average of a set of samples
        if (ch == 0 && ++stick_subtrim_cycles > GYRO_SUBTRIM_INIT_SAMPLES) {
            // Completed
            learn_state = GYRO_LEARN_OFF;
            return;
        }

        // Average over 10 cycles
        if (stick_subtrim_cycles < GYRO_SUBTRIM_INIT_SAMPLES) {
            auto ch_limit = &temp_limits[ch];
            ch_limit->val.mid = (((ch_limit->val.mid * stick_subtrim_cycles) / stick_subtrim_cycles) + us) / 2;
        }
    }
    else 
    if (learn_state== GYRO_LEARN_LIMIT_START) {
        auto ch_limit = &temp_limits[ch];

        if (us < ch_limit->val.min) {
            ch_limit->val.min = us;
        }
        if (us > ch_limit->val.max) {
            ch_limit->val.max = us;
        }
    }
}

void Gyro::StickCenterCalibration() {

    DBGLN("Gyro(): Stick Center Calibration (Init)");
    stick_subtrim_cycles = 0;

    //initialize min,max, mid
    for (int ch=0;ch<PWM_MAX_CHANNELS;ch++) {
        auto ch_limit = &temp_limits[ch];
        ch_limit->val.mid = GYRO_US_MID; 
        ch_limit->val.min = GYRO_US_MID;
        ch_limit->val.max = GYRO_US_MID;
    }

    learn_state = GYRO_LEARN_SUBTRIMS;
}

void Gyro::StickLimitCalibration(bool done)
{
   DBGLN("Gyro(): Stick Range Calibration (%s)",done?"Complete":"Started");

   if (done) {
        learn_state = GYRO_LEARN_LIMIT_DONE;
        // save the Range
        for (int ch=0;ch<PWM_MAX_CHANNELS;ch++) {
            auto ch_info = config.GetGyroChannel(ch);
            auto output_mode = (gyro_output_channel_function_t) ch_info->val.output_mode;
            if (output_mode!= FN_NONE) {
                auto pwm_limits =  &temp_limits[ch];
                DBGLN("Ch%d: Min: %d Max: %d Center: %d", 
                    ch, (uint16_t) pwm_limits->val.min, (uint16_t) pwm_limits->val.max, (uint16_t) pwm_limits->val.mid);
                config.SetPwmChannelLimitsRaw(ch,pwm_limits->raw);
            }
        }
        config.Commit();
   } else {
        learn_state = GYRO_LEARN_LIMIT_START;
   }
}

#endif
