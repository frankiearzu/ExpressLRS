#pragma once
#include "IMUBase.h"

class QMI8658C : public IMUBase {
public:
    QMI8658C() : IMUBase(0x6B) {}

    bool initialize();

protected:
    bool getRawData(FusionVector &accel, FusionVector &gyro);

private:
    int writeCommand(uint8_t cmd);
};