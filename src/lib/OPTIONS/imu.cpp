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
    {IMU_orient_z, "imu_orient_z", INT},
    {IMU_orient_y, "imu_orient_y", INT},
    {IMU_mod_ch_pos_n100, "imu_mod_ch_pos_n100", INT},
    {IMU_mod_ch_pos_n50, "imu_mod_ch_pos_n50", INT},
    {IMU_mod_ch_pos_0, "imu_mod_ch_pos_0", INT},
    {IMU_mod_ch_pos_50, "imu_mod_ch_pos_50", INT},
    {IMU_mod_ch_pos_100, "imu_mod_ch_pos_100", INT},
    {IMU_rate_sens, "imu_rate_sens", INT},
    {IMU_rate_mod_stick_prio, "imu_rate_mod_stick_prio", INT},
    {IMU_rate_mod_gain_rol, "imu_rate_mod_gain_rol", INT},
    {IMU_rate_mod_gain_pit, "imu_rate_mod_gain_pit", INT},
    {IMU_rate_mod_gain_yaw, "imu_rate_mod_gain_yaw", INT},
    {IMU_env_mod_use_rate, "imu_env_mod_use_rate", INT},
    {IMU_env_mod_max_rol, "imu_env_mod_max_rol", INT},
    {IMU_env_mod_max_pit, "imu_env_mod_max_pit", INT},
    {IMU_env_mod_gain_rol, "imu_env_mod_gain_rol", INT},
    {IMU_env_mod_gain_pit, "imu_env_mod_gain_pit", INT},
    {IMU_env_mod_gain_yaw, "imu_env_mod_gain_yaw", INT},
    {IMU_ang_mod_use_rate, "imu_ang_mod_use_rate", INT},
    {IMU_ang_mod_max_rol, "imu_ang_mod_max_rol", INT},
    {IMU_ang_mod_max_pit, "imu_ang_mod_max_pit", INT},
    {IMU_ang_mod_trim_rol, "imu_ang_mod_trim_rol", INT},
    {IMU_ang_mod_trim_pit, "imu_ang_mod_trim_pit", INT},
    {IMU_ang_mod_gain_rol, "imu_ang_mod_gain_rol", INT},
    {IMU_ang_mod_gain_pit, "imu_ang_mod_gain_pit", INT},

    {IMU_mod_ch_1_func, "imu_mod_ch_1_func", INT},
    {IMU_mod_ch_1_prim, "imu_mod_ch_1_prim", BOOL},
    {IMU_mod_ch_1_inv, "imu_mod_ch_1_inv", BOOL},

    {IMU_mod_ch_2_func, "imu_mod_ch_2_func", INT},
    {IMU_mod_ch_2_prim, "imu_mod_ch_2_prim", BOOL},
    {IMU_mod_ch_2_inv, "imu_mod_ch_2_inv", BOOL},

    {IMU_mod_ch_3_func, "imu_mod_ch_3_func", INT},
    {IMU_mod_ch_3_prim, "imu_mod_ch_3_prim", BOOL},
    {IMU_mod_ch_3_inv, "imu_mod_ch_3_inv", BOOL},

    {IMU_mod_ch_4_func, "imu_mod_ch_4_func", INT},
    {IMU_mod_ch_4_prim, "imu_mod_ch_4_prim", BOOL},
    {IMU_mod_ch_4_inv, "imu_mod_ch_4_inv", BOOL},

    {IMU_mod_ch_5_func, "imu_mod_ch_5_func", INT},
    {IMU_mod_ch_5_prim, "imu_mod_ch_5_prim", BOOL},
    {IMU_mod_ch_5_inv, "imu_mod_ch_5_inv", BOOL},

    {IMU_mod_ch_6_func, "imu_mod_ch_6_func", INT},
    {IMU_mod_ch_6_prim, "imu_mod_ch_6_prim", BOOL},
    {IMU_mod_ch_6_inv, "imu_mod_ch_6_inv", BOOL},

    {IMU_mod_ch_7_func, "imu_mod_ch_7_func", INT},
    {IMU_mod_ch_7_prim, "imu_mod_ch_7_prim", BOOL},
    {IMU_mod_ch_7_inv, "imu_mod_ch_7_inv", BOOL},

    {IMU_mod_ch_8_func, "imu_mod_ch_8_func", INT},
    {IMU_mod_ch_8_prim, "imu_mod_ch_8_prim", BOOL},
    {IMU_mod_ch_8_inv, "imu_mod_ch_8_inv", BOOL},

    {IMU_mod_ch_9_func, "imu_mod_ch_9_func", INT},
    {IMU_mod_ch_9_prim, "imu_mod_ch_9_prim", BOOL},
    {IMU_mod_ch_9_inv, "imu_mod_ch_9_inv", BOOL},

    {IMU_mod_ch_10_func, "imu_mod_ch_10_func", INT},
    {IMU_mod_ch_10_prim, "imu_mod_ch_10_prim", BOOL},
    {IMU_mod_ch_10_inv, "imu_mod_ch_10_inv", BOOL},

    {IMU_mod_ch_11_func, "imu_mod_ch_11_func", INT},
    {IMU_mod_ch_11_prim, "imu_mod_ch_11_prim", BOOL},
    {IMU_mod_ch_11_inv, "imu_mod_ch_11_inv", BOOL},

    {IMU_mod_ch_12_func, "imu_mod_ch_12_func", INT},
    {IMU_mod_ch_12_prim, "imu_mod_ch_12_prim", BOOL},
    {IMU_mod_ch_12_inv, "imu_mod_ch_12_inv", BOOL},

    {IMU_mod_ch_13_func, "imu_mod_ch_13_func", INT},
    {IMU_mod_ch_13_prim, "imu_mod_ch_13_prim", BOOL},
    {IMU_mod_ch_13_inv, "imu_mod_ch_13_inv", BOOL},

    {IMU_mod_ch_14_func, "imu_mod_ch_14_func", INT},
    {IMU_mod_ch_14_prim, "imu_mod_ch_14_prim", BOOL},
    {IMU_mod_ch_14_inv, "imu_mod_ch_14_inv", BOOL},

    {IMU_mod_ch_15_func, "imu_mod_ch_15_func", INT},
    {IMU_mod_ch_15_prim, "imu_mod_ch_15_prim", BOOL},
    {IMU_mod_ch_15_inv, "imu_mod_ch_15_inv", BOOL},

    {IMU_mod_ch_16_func, "imu_mod_ch_16_func", INT},
    {IMU_mod_ch_16_prim, "imu_mod_ch_16_prim", BOOL},
    {IMU_mod_ch_16_inv, "imu_mod_ch_16_inv", BOOL},
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
