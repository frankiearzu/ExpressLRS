#if !defined(UNIT_TEST)
#include "options.h"
#include "helpers.h"
#include "logging.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

typedef enum {
    INT,
    BOOL,
    FLOAT,
    ARRAY,
    COUNT
} imu_datatype_t;

static const struct {
    const imuNameType position;
    const char *name;
    const imu_datatype_t type;
} fields[] = {
    {IMU_customised, "imu_customised", BOOL},
    {IMU_disabled, "imu_disabled", BOOL},
    {IMU_orientation_z, "imu_orientation_z", INT},
    {IMU_orientation_y, "imu_orientation_y", INT},
    {IMU_mode_channel_position_n100, "imu_mode_channel_position_n100", INT},
    {IMU_mode_channel_position_n50, "imu_mode_channel_position_n50", INT},
    {IMU_mode_channel_position_0, "imu_mode_channel_position_0", INT},
    {IMU_mode_channel_position_50, "imu_mode_channel_position_50", INT},
    {IMU_mode_channel_position_100, "imu_mode_channel_position_100", INT},
    {IMU_rate_sensitivity, "imu_rate_sensitivity", INT},
    {IMU_rate_mode_stick_priority, "imu_rate_mode_stick_priority", INT},
    {IMU_rate_mode_gain_roll, "imu_rate_mode_gain_roll", INT},
    {IMU_rate_mode_gain_pitch, "imu_rate_mode_gain_pitch", INT},
    {IMU_rate_mode_gain_yaw, "imu_rate_mode_gain_yaw", INT},
    {IMU_envelope_mode_use_rate, "imu_envelope_mode_use_rate", INT},
    {IMU_envelope_mode_max_rol, "imu_envelope_mode_max_rol", INT},
    {IMU_envelope_mode_max_pit, "imu_envelope_mode_max_pit", INT},
    {IMU_envelope_mode_gain_rol, "imu_envelope_mode_gain_rol", INT},
    {IMU_envelope_mode_gain_pit, "imu_envelope_mode_gain_pit", INT},
    {IMU_envelope_mode_gain_yaw, "imu_envelope_mode_gain_yaw", INT},
    {IMU_angle_mode_use_rate, "imu_angle_mode_use_rate", INT},
    {IMU_angle_mode_max_rol, "imu_angle_mode_max_rol", INT},
    {IMU_angle_mode_max_pit, "imu_angle_mode_max_pit", INT},
    {IMU_angle_mode_trim_rol, "imu_angle_mode_trim_rol", INT},
    {IMU_angle_mode_trim_pit, "imu_angle_mode_trim_pit", INT},
    {IMU_angle_mode_gain_rol, "imu_angle_mode_gain_rol", INT},
    {IMU_angle_mode_gain_pit, "imu_angle_mode_gain_pit", INT},

    {IMU_channel_1_function, "imu_channel_1_function", INT},
    {IMU_channel_1_primary, "imu_channel_1_primary", BOOL},
    {IMU_channel_1_invert, "imu_channel_1_invert", BOOL},

    {IMU_channel_2_function, "imu_channel_2_function", INT},
    {IMU_channel_2_primary, "imu_channel_2_primary", BOOL},
    {IMU_channel_2_invert, "imu_channel_2_invert", BOOL},

    {IMU_channel_3_function, "imu_channel_3_function", INT},
    {IMU_channel_3_primary, "imu_channel_3_primary", BOOL},
    {IMU_channel_3_invert, "imu_channel_3_invert", BOOL},

    {IMU_channel_4_function, "imu_channel_4_function", INT},
    {IMU_channel_4_primary, "imu_channel_4_primary", BOOL},
    {IMU_channel_4_invert, "imu_channel_4_invert", BOOL},

    {IMU_channel_5_function, "imu_channel_5_function", INT},
    {IMU_channel_5_primary, "imu_channel_5_primary", BOOL},
    {IMU_channel_5_invert, "imu_channel_5_invert", BOOL},

    {IMU_channel_6_function, "imu_channel_6_function", INT},
    {IMU_channel_6_primary, "imu_channel_6_primary", BOOL},
    {IMU_channel_6_invert, "imu_channel_6_invert", BOOL},

    {IMU_channel_7_function, "imu_channel_7_function", INT},
    {IMU_channel_7_primary, "imu_channel_7_primary", BOOL},
    {IMU_channel_7_invert, "imu_channel_7_invert", BOOL},

    {IMU_channel_8_function, "imu_channel_8_function", INT},
    {IMU_channel_8_primary, "imu_channel_8_primary", BOOL},
    {IMU_channel_8_invert, "imu_channel_8_invert", BOOL},

    {IMU_channel_9_function, "imu_channel_9_function", INT},
    {IMU_channel_9_primary, "imu_channel_9_primary", BOOL},
    {IMU_channel_9_invert, "imu_channel_9_invert", BOOL},

    {IMU_channel_10_function, "imu_channel_10_function", INT},
    {IMU_channel_10_primary, "imu_channel_10_primary", BOOL},
    {IMU_channel_10_invert, "imu_channel_10_invert", BOOL},

    {IMU_channel_11_function, "imu_channel_11_function", INT},
    {IMU_channel_11_primary, "imu_channel_11_primary", BOOL},
    {IMU_channel_11_invert, "imu_channel_11_invert", BOOL},

    {IMU_channel_12_function, "imu_channel_12_function", INT},
    {IMU_channel_12_primary, "imu_channel_12_primary", BOOL},
    {IMU_channel_12_invert, "imu_channel_12_invert", BOOL},

    {IMU_channel_13_function, "imu_channel_13_function", INT},
    {IMU_channel_13_primary, "imu_channel_13_primary", BOOL},
    {IMU_channel_13_invert, "imu_channel_13_invert", BOOL},

    {IMU_channel_14_function, "imu_channel_14_function", INT},
    {IMU_channel_14_primary, "imu_channel_14_primary", BOOL},
    {IMU_channel_14_invert, "imu_channel_14_invert", BOOL},

    {IMU_channel_15_function, "imu_channel_15_function", INT},
    {IMU_channel_15_primary, "imu_channel_15_primary", BOOL},
    {IMU_channel_15_invert, "imu_channel_15_invert", BOOL},

    {IMU_channel_16_function, "imu_channel_16_function", INT},
    {IMU_channel_16_primary, "imu_channel_16_primary", BOOL},
    {IMU_channel_16_invert, "imu_channel_16_invert", BOOL},

    {IMU_rate_roll_gain_p, "imu_rate_roll_gain_p", INT},
    {IMU_rate_roll_gain_i, "imu_rate_roll_gain_i", INT},
    {IMU_rate_roll_gain_d, "imu_rate_roll_gain_d", INT},
    {IMU_rate_pitch_gain_p, "imu_rate_pitch_gain_p", INT},
    {IMU_rate_pitch_gain_i, "imu_rate_pitch_gain_i", INT},
    {IMU_rate_pitch_gain_d, "imu_rate_pitch_gain_d", INT},
    {IMU_rate_yaw_gain_p, "imu_rate_yaw_gain_p", INT},
    {IMU_rate_yaw_gain_i, "imu_rate_yaw_gain_i", INT},
    {IMU_rate_yaw_gain_d, "imu_rate_yaw_gain_d", INT},

    {IMU_envelope_roll_gain_p, "imu_envelope_roll_gain_p", INT},
    {IMU_envelope_roll_gain_i, "imu_envelope_roll_gain_i", INT},
    {IMU_envelope_roll_gain_d, "imu_envelope_roll_gain_d", INT},
    {IMU_envelope_pitch_gain_p, "imu_envelope_pitch_gain_p", INT},
    {IMU_envelope_pitch_gain_i, "imu_envelope_pitch_gain_i", INT},
    {IMU_envelope_pitch_gain_d, "imu_envelope_pitch_gain_d", INT},
    {IMU_envelope_yaw_gain_p, "imu_envelope_yaw_gain_p", INT},
    {IMU_envelope_yaw_gain_i, "imu_envelope_yaw_gain_i", INT},
    {IMU_envelope_yaw_gain_d, "imu_envelope_yaw_gain_d", INT},
};

typedef union {
    int int_value;
    bool bool_value;
    float float_value;
    int16_t *array_value;
} imu_data_holder_t;

static imu_data_holder_t imu[IMU_LAST];
static String builtinIMUConfig;
static String defaultIMUConfig = "{\"imu_disabled\":true}";

String& getIMU()
{
    File file = LittleFS.open("/imu.json", "r");
    if (!file || file.isDirectory())
    {
        if (file)
        {
            file.close();
        }
        // Return default IMU config
        return defaultIMUConfig;
    }
    builtinIMUConfig = file.readString();
    return builtinIMUConfig;
}

static void imu_ClearAllFields()
{
    for (auto field : fields) {
        switch (field.type) {
            case INT:
                imu[field.position].int_value = -1;
                break;
            case BOOL:
                imu[field.position].bool_value = false;
                break;
            case FLOAT:
                imu[field.position].float_value = 0.0;
                break;
            case ARRAY:
                imu[field.position].array_value = nullptr;
                break;
            case COUNT:
                imu[field.position].int_value = 0;
                break;
        }
    }
}

static void imu_LoadFieldsFromDoc(JsonDocument &doc)
{
    for (auto field : fields) {
        if (doc[field.name].is<JsonVariant>()) {
            switch (field.type) {
                case INT:
                    imu[field.position].int_value = doc[field.name];
                    break;
                case BOOL:
                    imu[field.position].bool_value = doc[field.name];
                    break;
                case FLOAT:
                    imu[field.position].float_value = doc[field.name];
                    break;
                case ARRAY:
                    {
                        JsonArray array = doc[field.name].as<JsonArray>();
                        imu[field.position].array_value = new int16_t[array.size()];
                        copyArray(array, imu[field.position].array_value, array.size());
                    }
                    break;
                case COUNT:
                    {
                        JsonArray array = doc[field.name].as<JsonArray>();
                        imu[field.position].int_value = (int)array.size();
                    }
                    break;
            }
        }
    }
}

int imu_pin(nameType name)
{
    return imu[name].int_value;
}

bool imu_flag(nameType name)
{
    return imu[name].bool_value;
}

int imu_int(nameType name)
{
    return imu[name].int_value;
}

float imu_float(nameType name)
{
    return imu[name].float_value;
}

const int16_t* imu_i16_array(nameType name)
{
    return imu[name].array_value;
}

const uint16_t* imu_u16_array(nameType name)
{
    return (uint16_t *)imu[name].array_value;
}
#endif
