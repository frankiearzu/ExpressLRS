#include <stdint.h>

typedef enum {
    // IMU customisation flag
    IMU_customised,

    IMU_disabled,
    IMU_orientation_z,
    IMU_orientation_y,
    IMU_mode_channel_position_n100,
    IMU_mode_channel_position_n50,
    IMU_mode_channel_position_0,
    IMU_mode_channel_position_50,
    IMU_mode_channel_position_100,
    IMU_rate_sensitivity,
    IMU_rate_mode_stick_priority,
    IMU_rate_mode_gain_roll,
    IMU_rate_mode_gain_pitch,
    IMU_rate_mode_gain_yaw,
    IMU_envelope_mode_use_rate,
    IMU_envelope_mode_max_rol,
    IMU_envelope_mode_max_pit,
    IMU_envelope_mode_gain_rol,
    IMU_envelope_mode_gain_pit,
    IMU_envelope_mode_gain_yaw,
    IMU_angle_mode_use_rate,
    IMU_angle_mode_max_rol,
    IMU_angle_mode_max_pit,
    IMU_angle_mode_trim_rol,
    IMU_angle_mode_trim_pit,
    IMU_angle_mode_gain_rol,
    IMU_angle_mode_gain_pit,

    IMU_channel_1_function,
    IMU_channel_1_primary,
    IMU_channel_1_invert,

    IMU_channel_2_function,
    IMU_channel_2_primary,
    IMU_channel_2_invert,

    IMU_channel_3_function,
    IMU_channel_3_primary,
    IMU_channel_3_invert,

    IMU_channel_4_function,
    IMU_channel_4_primary,
    IMU_channel_4_invert,

    IMU_channel_5_function,
    IMU_channel_5_primary,
    IMU_channel_5_invert,

    IMU_channel_6_function,
    IMU_channel_6_primary,
    IMU_channel_6_invert,

    IMU_channel_7_function,
    IMU_channel_7_primary,
    IMU_channel_7_invert,

    IMU_channel_8_function,
    IMU_channel_8_primary,
    IMU_channel_8_invert,

    IMU_channel_9_function,
    IMU_channel_9_primary,
    IMU_channel_9_invert,

    IMU_channel_10_function,
    IMU_channel_10_primary,
    IMU_channel_10_invert,

    IMU_channel_11_function,
    IMU_channel_11_primary,
    IMU_channel_11_invert,

    IMU_channel_12_function,
    IMU_channel_12_primary,
    IMU_channel_12_invert,

    IMU_channel_13_function,
    IMU_channel_13_primary,
    IMU_channel_13_invert,

    IMU_channel_14_function,
    IMU_channel_14_primary,
    IMU_channel_14_invert,

    IMU_channel_15_function,
    IMU_channel_15_primary,
    IMU_channel_15_invert,

    IMU_channel_16_function,
    IMU_channel_16_primary,
    IMU_channel_16_invert,

    IMU_LAST
} imuNameType;

int imu_pin(imuNameType name);
bool imu_flag(imuNameType name);
int imu_int(imuNameType name);
float imu_float(imuNameType name);
const int16_t* imu_i16_array(imuNameType name);
const uint16_t* imu_u16_array(imuNameType name);
