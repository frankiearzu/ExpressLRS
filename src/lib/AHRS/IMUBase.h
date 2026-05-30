#pragma once

class IMUBase {
public:
    virtual ~IMUBase() = default;

    virtual bool initialize() = 0;
    void setInterruptHandler(int pin);


    int getSampleRate() const { return sampleRate; }
    float getGyroRange() const { return gyroRange; }

    virtual bool isDataReady() = 0;
    virtual bool getRawData(int16_t *ax, int16_t *ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t *gz) = 0;
protected:
    int address;
    int sampleRate = 0;
    float gyroRange;

    virtual void writeRegister(uint8_t reg, uint8_t val);
    virtual uint8_t readRegister(uint8_t reg);
    virtual uint8_t readRegistersBuffer(uint8_t reg, uint8_t *buffer, int length);
};

