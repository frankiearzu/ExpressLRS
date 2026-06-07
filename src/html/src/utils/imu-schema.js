const IMU_SCHEMA = [
    {
        title: 'General',
        sections: [
            {
                title: '', rows: [
                    {
                        id: 'imu_disabled',
                        label: 'Disable IMU',
                        type: 'checkbox',
                        icon: null,
                        desc: 'Ignore the IMU presence when the receiver starts'
                    }
                ]
            }
        ]
    },
    {
        title: 'Setup',
        sections: [
            {
                title: 'Receiver orientation', rows: [
                    {
                        id: 'imu_orientation_z',
                        label: 'Label orientation',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Unknown'},
                            {value: 1, label: 'Up'},
                            {value: 2, label: 'Down'},
                            {value: 3, label: 'Port side'},
                            {value: 4, label: 'Starboard side'},
                            {value: 5, label: 'Forward'},
                            {value: 6, label: 'Aft'}
                        ], desc: 'Orientation of the receiver label'
                    },
                    {
                        id: 'imu_orientation_y',
                        label: 'PWM pins orientation',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Unknown'},
                            {value: 1, label: 'Up'},
                            {value: 2, label: 'Down'},
                            {value: 3, label: 'Port side'},
                            {value: 4, label: 'Starboard side'},
                            {value: 5, label: 'Forward'},
                            {value: 6, label: 'Aft'}
                        ], desc: 'Orientation of the receiver PWM pins'
                    },
                    {
                        type: 'spacer'
                    },
                ]
            },
            {
                title: 'Channel functions', rows: _generateChannels(),
            },
            {
                title: 'Flight mode switch', rows: [
                    {
                        id: 'imu_mode_channel_position_n100',
                        label: 'Position -100%',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Off'},
                            {value: 1, label: 'Rate'},
                            {value: 2, label: 'Envelope'},
                            {value: 3, label: 'Auto-level'}
                        ], desc: 'Flight mode selected when the channel assigned to the flight mode selection has value -100%'
                    },
                    {
                        id: 'imu_mode_channel_position_n50',
                        label: 'Position -50%',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Off'},
                            {value: 1, label: 'Rate'},
                            {value: 2, label: 'Envelope'},
                            {value: 3, label: 'Auto-level'}
                        ], desc: 'Flight mode selected when the channel assigned to the flight mode selection has value -50%'
                    },
                    {
                        id: 'imu_mode_channel_position_0',
                        label: 'Position 0',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Off'},
                            {value: 1, label: 'Rate'},
                            {value: 2, label: 'Envelope'},
                            {value: 3, label: 'Auto-level'}
                        ], desc: 'Flight mode selected when the channel assigned to the flight mode selection has value zero'
                    },
                    {
                        id: 'imu_mode_channel_position_50',
                        label: 'Position 50%',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Off'},
                            {value: 1, label: 'Rate'},
                            {value: 2, label: 'Envelope'},
                            {value: 3, label: 'Auto-level'}
                        ], desc: 'Flight mode selected when the channel assigned to the flight mode selection has value 50%'
                    },
                    {
                        id: 'imu_mode_channel_position_100',
                        label: 'Position 100%',
                        type: 'select',
                        options: [
                            {value: 0, label: 'Off'},
                            {value: 1, label: 'Rate'},
                            {value: 2, label: 'Envelope'},
                            {value: 3, label: 'Auto-level'}
                        ], desc: 'Flight mode selected when the channel assigned to the flight mode selection has value 100%'
                    }
                ]
            },
        ]
    },
    {
        title: 'Flight modes',
        sections: [
            {
                title: 'Rate mode', rows: [
                    {
                        id: 'imu_rate_sensitivity',
                        label: 'Gain sensitivity',
                        type: 'select',
                        options: [
                            {value: 0, label: '0.5x'},
                            {value: 1, label: ' 1x'},
                            {value: 2, label: ' 2x'},
                        ], desc: 'Increase or decrease the sensitivity. Acts as a multipler for all gains.'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_rate_mode_stick_priority',
                        label: 'Stick priority',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'The gains decrese linearly as the sticks move away from center. This is the fraction of total stick movement where the gains reach zero.'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_rate_mode_gain_roll',
                        label: 'Roll gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to stabilize the aircraft. (30% is a good start.)'
                    },
                    {
                        id: 'imu_rate_mode_gain_pitch',
                        label: 'Pitch gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to stabilize the aircraft. (40% is a good start.)'
                    },
                    ,
                    {
                        id: 'imu_rate_mode_gain_yaw',
                        label: 'Yaw gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to stabilize the aircraft. (50% is a good start.)'
                    },
                    {
                        type: 'spacer'
                    },
                ]
            },
            {
                title: 'Envelope mode', rows: [
                    {
                        id: 'imu_envelope_mode_use_rate',
                        label: 'Use Rate',
                        type: 'checkbox',
                        desc: 'Use in combination with Rate mode for stabilization'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_envelope_mode_max_rol',
                        label: 'Max roll angle',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Maximum bank angle'
                    },
                    {
                        id: 'imu_envelope_mode_max_pit',
                        label: 'Max pitch angle',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Maximum pitch angle'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_envelope_mode_gain_rol',
                        label: 'Roll gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to keep the aircraft at max angle. (35% is a good start for soft movement.)'
                    },
                    {
                        id: 'imu_envelope_mode_gain_pit',
                        label: 'Pitch gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to keep the aircraft at max angle. (35% is a good start for soft movement.)'
                    },
                    ,
                    {
                        id: 'imu_envelope_mode_gain_yaw',
                        label: 'Yaw gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to keep the aircraft at max angle. (35% is a good start for soft movement.)'
                    },
                    {
                        type: 'spacer'
                    },
                ]
            },
            {
                title: 'Auto-level mode', rows: [
                    {
                        id: 'imu_angle_mode_use_rate',
                        label: 'Use Rate',
                        type: 'checkbox',
                        desc: 'Use in combination with Rate mode for stabilization'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_angle_mode_max_rol',
                        label: 'Max roll angle',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Maximum bank angle'
                    },
                    {
                        id: 'imu_angle_mode_max_pit',
                        label: 'Max pitch angle',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Maximum pitch angle'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_angle_mode_trim_rol',
                        label: 'Roll angle trim',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Positive trim lowers the left wing'
                    },
                    {
                        id: 'imu_angle_mode_trim_pit',
                        label: 'Pitch angle trim',
                        type: 'uint',
                        unit: 'deg',
                        icon: '',
                        desc: 'Positive trim pitches the nose up'
                    },
                    {
                        type: 'spacer'
                    },
                    {
                        id: 'imu_angle_mode_gain_rol',
                        label: 'Roll gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to return the aircraft to level. (35% is a good start for soft movement.)'
                    },
                    {
                        id: 'imu_angle_mode_gain_pit',
                        label: 'Pitch gain',
                        type: 'uint',
                        unit: '%',
                        icon: '',
                        desc: 'Fraction of total movement used to return the aircraft to level. (35% is a good start for soft movement.)'
                    }
                ]
            }
        ]
    }
];

function _generateChannels() {
    const fields = [];

    [...Array(16).keys()].forEach((i) => {
        const channelNumber = i+1
        const channelName = `CH${channelNumber}`
        fields.push({
            id: `imu_channel_${channelNumber}_function`,
            label: `${channelName}`,
            type: 'select',
            options: [
                {value: 0, label: 'None'},
                {value: 1, label: 'Aileron'},
                {value: 2, label: 'Elevator'},
                {value: 3, label: 'Rudder'},
                {value: 4, label: 'Elevon Port'},
                {value: 5, label: 'Elevon Starboard'},
                {value: 6, label: 'V-Tail Port'},
                {value: 7, label: 'V-Tail Starboard'},
                {value: 8, label: 'Flight Mode'},
                {value: 9, label: 'Gain'},
            ], desc: `Function assigned to ${channelName}`
        });
        fields.push({
            id: `imu_channel_${channelNumber}_primary`,
            label: `${channelName} is primary`,
            type: 'checkbox',
            desc: 'When multiple channels share a function, the primary will be used for calculations.'
        });
        fields.push({
            id: `imu_channel_${channelNumber}_invert`,
            label: `Invert output to ${channelName}`,
            type: 'checkbox',
            desc: 'If the IMU is correcting in the wrong direction, invert its output.'
        });
        fields.push({
            type: 'spacer',
        });
    });
    return fields;
}

export default IMU_SCHEMA;
