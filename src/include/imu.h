#include <stdint.h>

typedef enum {
    // IMU customisation flag
    IMU_customised,

    IMU_disabled,
    IMU_orient_z,
    IMU_orient_y,
    IMU_mod_ch_pos_n100,
    IMU_mod_ch_pos_n50,
    IMU_mod_ch_pos_0,
    IMU_mod_ch_pos_50,
    IMU_mod_ch_pos_100,
    IMU_rate_sens,
    IMU_rate_mod_stick_prio,
    IMU_rate_mod_gain_rol,
    IMU_rate_mod_gain_pit,
    IMU_rate_mod_gain_yaw,
    IMU_env_mod_use_rate,
    IMU_env_mod_max_rol,
    IMU_env_mod_max_pit,
    IMU_env_mod_gain_rol,
    IMU_env_mod_gain_pit,
    IMU_env_mod_gain_yaw,
    IMU_ang_mod_use_rate,
    IMU_ang_mod_max_rol,
    IMU_ang_mod_max_pit,
    IMU_ang_mod_trim_rol,
    IMU_ang_mod_trim_pit,
    IMU_ang_mod_gain_rol,
    IMU_ang_mod_gain_pit,


    IMU_mod_ch_1_func,
    IMU_mod_ch_1_prim,
    IMU_mod_ch_1_inv,

    IMU_mod_ch_2_func,
    IMU_mod_ch_2_prim,
    IMU_mod_ch_2_inv,

    IMU_mod_ch_3_func,
    IMU_mod_ch_3_prim,
    IMU_mod_ch_3_inv,

    IMU_mod_ch_4_func,
    IMU_mod_ch_4_prim,
    IMU_mod_ch_4_inv,

    IMU_mod_ch_5_func,
    IMU_mod_ch_5_prim,
    IMU_mod_ch_5_inv,

    IMU_mod_ch_6_func,
    IMU_mod_ch_6_prim,
    IMU_mod_ch_6_inv,

    IMU_mod_ch_7_func,
    IMU_mod_ch_7_prim,
    IMU_mod_ch_7_inv,

    IMU_mod_ch_8_func,
    IMU_mod_ch_8_prim,
    IMU_mod_ch_8_inv,

    IMU_mod_ch_9_func,
    IMU_mod_ch_9_prim,
    IMU_mod_ch_9_inv,

    IMU_mod_ch_10_func,
    IMU_mod_ch_10_prim,
    IMU_mod_ch_10_inv,

    IMU_mod_ch_11_func,
    IMU_mod_ch_11_prim,
    IMU_mod_ch_11_inv,

    IMU_mod_ch_12_func,
    IMU_mod_ch_12_prim,
    IMU_mod_ch_12_inv,

    IMU_mod_ch_13_func,
    IMU_mod_ch_13_prim,
    IMU_mod_ch_13_inv,

    IMU_mod_ch_14_func,
    IMU_mod_ch_14_prim,
    IMU_mod_ch_14_inv,

    IMU_mod_ch_15_func,
    IMU_mod_ch_15_prim,
    IMU_mod_ch_15_inv,

    IMU_mod_ch_16_func,
    IMU_mod_ch_16_prim,
    IMU_mod_ch_16_inv,

    IMU_LAST
} imuNameType;

int imu_pin(imuNameType name);
bool imu_flag(imuNameType name);
int imu_int(imuNameType name);
float imu_float(imuNameType name);
const int16_t* imu_i16_array(imuNameType name);
const uint16_t* imu_u16_array(imuNameType name);
