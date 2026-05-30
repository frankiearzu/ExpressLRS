#include <Arduino.h>
#include <Wire.h>
#include "logging.h"
#include "IMUBase.h"


static volatile bool irq_received;

static IRAM_ATTR void irq_handler() {
    irq_received = true;
}

void IMUBase::setInterruptHandler(int pin) {
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, irq_handler, RISING);
}

void IMUBase::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t IMUBase::readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    return Wire.read();
}

uint8_t IMUBase::readRegistersBuffer(uint8_t reg, uint8_t *buffer, int length) {
    uint32_t t1 = millis();
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, length);
    int count = 0;
    while (Wire.available() && (millis() - t1 < 1000) && count < length) {
        buffer[count++] = Wire.read();
    }
    return count;
}

bool IMUBase::isDataReady() {
    if (irq_received) return true;
    return false;
}

bool IMUBase::getRawData(int16_t *ax, int16_t *ay, int16_t* az, int16_t* gz, int16_t* gy, int16_t *gz) {
    
    irq_received = false;
    return true;
}