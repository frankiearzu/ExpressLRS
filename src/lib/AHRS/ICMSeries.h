#pragma once

#include "FusionMath.h"
#include "IMUBase.h"

class ICMSeries : public IMUBase {
public:
    ICMSeries() : IMUBase(0x68) {}
    bool initialize();

protected:
    bool getRawData(FusionVector &accel, FusionVector &gyro);

private:
    void writeMem1Register(uint8_t reg, uint8_t val);
};
