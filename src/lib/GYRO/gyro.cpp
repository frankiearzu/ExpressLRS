#include "targets.h"

#if defined(HAS_GYRO)
#include "config.h"
#include "gyro.h"
#include "gyro_types.h"
#include "mixer.h"
//#include "device.h"
#include "mpu/mpu.h"
#include "modes/mode_envelope.h"
#include "modes/mode_auto_level.h"
#include "modes/mode_hover.h"
#include "modes/mode_rate.h"
#include "CRSFRouter.h"
#include "logging.h"


// Comment to Remove Debug of State
#if defined(DEBUG_LOG)
#define GYRO_PID_DEBUG_TIME 5000  // Time im Ms

#ifdef GYRO_PID_DEBUG_TIME
unsigned long gyro_debug_time = 0;
#endif

#endif // DEBUG_LOG

// Channel Data
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];

#define GYRO_SUBTRIM_INIT_SAMPLES 10
static uint8_t stick_subtrim_cycles = 0;
static rx_config_pwm_limits_t temp_limits[PWM_MAX_CHANNELS] = {};

// Must match mixer.h: gyro_input_channel_function_t
//static const char* STR_gyroInputChannelMode[] = {"None","Roll","Pitch","Yaw","Mode","Gain"};
// Must match mixer.h: gyro_output_channel_function_t
//static const char* STR_gyroOutputChannelMode[] = {"None","Aileron","Elevator","Rudder","Elevon","V Tail"};
// Must match gyro.h gyro_mode_t
static const char* STR_gyroMode[] = {"Off","Rate","Envelope","Auto-Level","Launch","Hover"};
// Must match gyro_axis_t
static const char* STR_gyroAxis[] = {"Roll","Pitch","Yaw"}; 

//volatile gyro_event_t gyro_event = GYRO_EVENT_NONE;

static Mode_Base*  mode_controllers [GYRO_MODE_LAST_ACTIVE+1] = { };
static bool        first_start = true;

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
static int8_t GetGyroFunChannelNumber(gyro_output_channel_function_t mode, gyro_output_channel_function_t mode2 = (gyro_output_channel_function_t) 100, uint8_t start_ch = 0)
{
    int8_t result = -1;
    for (int8_t i = start_ch; i < GYRO_MAX_CHANNELS; i++) {
        auto info =  config.GetGyroChannel(i);
        if (info->val.output_mode == mode ||
            info->val.output_mode == mode2) {
            if (result==-1) {
                result = i;  // Minumun Ch that is that mode, check if it is the master
            }
            if (info->val.master) {
                // Found the Master, no need to look for more
                return i;
            }
        }
    }
    return result;
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
    if (crsfVal==CRSF_CHANNEL_VALUE_UNSET) return 0;
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

    mode_controllers[GYRO_MODE_OFF]      = nullptr;
    mode_controllers[GYRO_MODE_RATE]     = new RateController();
    mode_controllers[GYRO_MODE_LEVEL]    = new LevelController();
    mode_controllers[GYRO_MODE_LAUNCH]   = mode_controllers[GYRO_MODE_LEVEL];
    mode_controllers[GYRO_MODE_ENVELOPE] = new AngEnvelopeController();
    mode_controllers[GYRO_MODE_HOVER]    = new HoverController();
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
    initialized = mpuDev->isRunning() && !isStickCalibrationNeeded();

    gain_factor = 1.0;
    gyro_gain_factor_t gainFactorEnum = config.GetGyroGainFactor();
    switch (gainFactorEnum) {
        case GYRO_GAIN_FACTOR_0_5X: gain_factor = 0.5; break;
        case GYRO_GAIN_FACTOR_1X: gain_factor = 1.0; break;
        case GYRO_GAIN_FACTOR_1_5X: gain_factor = 1.5; break;
        case GYRO_GAIN_FACTOR_2X: gain_factor = 2; break;
    } 

    mode_controller = nullptr;
    mode_ch     = GetGyroFunChannelNumber(FN_GYRO_MODE);
    gain_ch     = GetGyroFunChannelNumber(FN_GYRO_GAIN);
    roll_ch     = GetGyroFunChannelNumber(FN_AILERON);
    pitch_ch    = GetGyroFunChannelNumber(FN_ELEVATOR);
    yaw_ch      = GetGyroFunChannelNumber(FN_RUDDER);
    
    elevon1_ch  = GetGyroFunChannelNumber(FN_ELEVON, FN_ELEVON_R);
    if (elevon1_ch >= 0) {
        elevon2_ch  = GetGyroFunChannelNumber(FN_ELEVON, FN_ELEVON_R, elevon1_ch + 1);
    }

    vtail1_ch     = GetGyroFunChannelNumber(FN_VTAIL, FN_VTAIL_R);
    if (vtail1_ch >= 0) {
        vtail2_ch = GetGyroFunChannelNumber(FN_VTAIL, FN_VTAIL_R, vtail1_ch + 1);
    }

    #ifdef GYRO_BOOT_JITTER
    if (first_start) {
        boot_jitter_times = 0;
        boot_jitter_time = 0;
    }
    #endif
    
    DBGLN("Gyro:Start() END");
    first_start = false;
}

const char * Gyro::getMPUName() {
    if (mpuDev == nullptr) return "--";
    return mpuDev->GetMPUName();
}

gyro_status_t Gyro::getStatus() 
{
    if (!config.GetGyroEnabled()) return GYRO_STATUS_OFF;
    if (mpuDev== nullptr) return GYRO_STATUS_NOT_DETECTED;
    if (!mpuDev->isRunning()) return GYRO_STATUS_NEED_RX_ORIENTATION; 
    if (isStickCalibrationNeeded()) return GYRO_STATUS_NEED_STICK_CAL;
    return GYRO_STATUS_OK;
}

void Gyro::calibrate()
{
    initialized = false;
    first_start = true;
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
    DBGLN("Gyro: Master Gain=[%f] * Gain_Factor=[%f] = %f", master_gain, gain_factor, master_gain * gain_factor);

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
    if (!initialized || learn_state != GYRO_LEARN_OFF || mpuDev->calibrating) return;


    if ((micros() - pid_delay) < 1000 ) return; // ~1k PID loop
    pid_delay = micros();

    if (mode_ch >= 0) detect_mode(channel_us(mode_ch));
    if (mode_controller == nullptr) return;

    //if (data_ready==0) return;
    //data_ready = 0;

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

    // ELEVON LOGIC if no aileron/elevator
    if (roll_ch == -1 && pitch_ch == -1 && elevon1_ch >= 0 && elevon2_ch >= 0) {
        auto i1 =  config.GetGyroChannel(elevon1_ch);
        auto i2 =  config.GetGyroChannel(elevon2_ch);

        auto e1  = channel_command(elevon1_ch) * ((i1->val.inverted)?-1:+1);
        auto e2  = channel_command(elevon2_ch) * ((i2->val.inverted)?-1:+1);

        // In the Radio, the Elevons are +50% ele, and +/-50% aileron
        // Pitch: The average of the two elevons, since both moves in the same direction.
        //          This gives a new "center".  So (e1 + e2) / 2.  Since the TX mix weight is 50%, then x2
        //          (e1 + e2) / 2 * 2 = (e1 + e2).
        // Roll:  Is how far e1 moved from the new "pitch" center, e1 x 2 to compensate for the 50% weight:  
        //          2 * e1 - (e1+e2) = 2*e1 -e1 - e2 = (e1-e2) 

        input_rpy[GYRO_AXIS_PITCH] = (e1 + e2);
        input_rpy[GYRO_AXIS_ROLL] = -(e1 - e2);
        
        // TODO? Do we need to invet roll??  ((i1->val.output_mode==FN_ELEVON_R)?-1:+1);
        
    } 

    if (yaw_ch == -1 && pitch_ch == 1 && vtail1_ch >= 0 && vtail2_ch >= 0) {
        // Try VTail
        auto i1 =  config.GetGyroChannel(vtail1_ch);
        auto i2 =  config.GetGyroChannel(vtail2_ch);

        auto v1  = channel_command(vtail1_ch) * ((i1->val.inverted)?-1:+1);
        auto v2  = channel_command(vtail2_ch) * ((i2->val.inverted)?-1:+1);;

        input_rpy[GYRO_AXIS_PITCH] = (v1 + v2);
        input_rpy[GYRO_AXIS_YAW]   = -(v1 - v2);
        
        //TODO? Do we need to invert YAW??  ((i1->val.output_mode==FN_VTAIL_R)?-1:+1);
    }



    if (gain_ch >= 0)   detect_gain(channel_us(gain_ch)); else master_gain = 1.0;

    mode_controller->calculate_pid(input_rpy, acc_rpy, angle_rpy);

    #if defined(DEBUG_LOG) && defined(GYRO_PID_DEBUG_TIME)
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
    if (!initialized || mpuDev->calibrating) return;
   
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

unsigned long Gyro::getIMUReadErrors() {
    return (mpuDev==nullptr)?0:mpuDev->read_errors;
}

int Gyro::tick()
{
    // Behaves like Global
    static unsigned long last_tel = millis(); 
    static unsigned long last_tick = micros();

    if (!initialized ||
        mpuDev->calibrating) { 
        //DBGLN("Gyro not Ready or Calibrating.. return in 1s");
        return 1000; // come back in 1000 ms if not initialized
    }

    // only try to check data ready every 50uS 
    long now = micros();
    if (now - last_tick < 50) {   
         return DURATION_IMMEDIATELY;
    }
    last_tick = now;
    
    // Do we have Gyro data Available ??
    if (!mpuDev->isDataReady()) return DURATION_IMMEDIATELY;

    if (mpuDev->read(acc_rpy, angle_rpy)) {  
        if ((millis() - last_tel) > 200 ) { // 200 ms (2s) cycle
            last_tel = millis();
            send_telemetry();
        }
    } else {
        // Read Error, try again inmediatly
        return DURATION_IMMEDIATELY;
    }

    // Loop again as fast as we can, the refresh rate of the gyro
    // is driven by the hardware ouput-data-rate (ODR)
    return DURATION_IMMEDIATELY;
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
            if (output_mode!= FN_NONE && output_mode != FN_GYRO_GAIN && output_mode != FN_GYRO_MODE) {
                // Only moving surfaces
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

bool Gyro::isStickCalibrationNeeded() {
    bool isCalibrated = true;
    //DBGLN("IsStickCalibrationNeeded: Start");

    for (int ch=0;ch < PWM_MAX_CHANNELS; ch++) {
            auto ch_info = config.GetGyroChannel(ch);
            auto output_mode = (gyro_output_channel_function_t) ch_info->val.output_mode;
            auto limits =  config.GetPwmChannelLimits(ch);
            if (output_mode!= FN_NONE && output_mode != FN_GYRO_GAIN && output_mode != FN_GYRO_MODE) {
                // Only valid surfaces are checked
                if ((limits->val.max == GYRO_US_MAX && limits->val.min == GYRO_US_MIN) ||  // Default
                    (limits->val.max == limits->val.min)) { // Not moved the sticks
                    DBGLN("isStickCalibrationNeeded: Ch [%d] Not Calibrated",ch+1);
                    isCalibrated = false;
                    break;
                }
            }
    }

    //DBGLN("IsStickCalibrationNeeded: All Calibrated");
    return ! isCalibrated;
}

#endif
