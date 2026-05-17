#pragma once
#if defined(HAS_GYRO)

#include "mixer.h"
#include "gyro.h"
#include "mode_rate.h"


class HoverController: public RateController
{
    public:
        void    initialize(gyro_mode_t mode);
        void    calculate_pid(float input_rpy[], float acc_rpy[], float ang_rpy[]);
        #if defined(DEBUG_LOG)
        void    printState();
        #endif
    protected:
        rx_config_gyro_fmode_t fm_angle_settings;
        float hoverStrengthPitch;
        float hoverStrengthYaw;

        float   errorPitch;
        float   errorYaw;
};

#endif