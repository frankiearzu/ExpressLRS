#pragma once
#include "mixer.h"
#include "gyro.h"
#include "mode_rate.h"

class LevelController: public RateController
{
    public:
        LevelController();
        void    initialize(gyro_mode_t mode);
        void    calculate_pid(float input_rpy[], float acc_rpy[], float ang_rpy[]);
        void    printState();

    protected:
        rx_config_gyro_fmode_t fm_angle_settings;
        PID pid_angle_roll, pid_angle_pitch;
};


