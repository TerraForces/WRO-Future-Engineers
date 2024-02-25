#include "MPU6050.h"

void mpuThreadFunction(void* parameter) {
    while(true) {
        uint64_t lastMicros = 0;
        if(micros() - lastMicros >= 1000000 / mpu6050.sampleRate){
            lastMicros = micros();
            for(uint8_t i = 0; i < 6; i++) {
                if(mpu6050.updateData & (1 << i)) {
                    mpu6050.update(i);
                }
            }
        }
    }
}

void MPU6050_Class::init(TwoWire* i2c, uint8_t address, uint8_t dataToUpdate, bool core, uint16_t samplesPerSecond, uint16_t calibrationSamples) {
    _address = address;
    _i2c = i2c;
    updateData = dataToUpdate;
    sampleRate = samplesPerSecond;
    writeRegister(0x19, (uint8_t)((1000.0 / sampleRate) - 1));
    writeRegister(0x1a, 0x1);
    writeRegister(0x1b, 0x0);
    writeRegister(0x1c, 0x0);
    writeRegister(0x6b, 0x1);
    delay(40);
    if(calibrationSamples > 0){
        for(uint8_t i = 0; i < 6; i++) {
            if(updateData & (1 << i)) {
                calibrate(i, calibrationSamples);
            }
        }
    }
    
    xTaskCreatePinnedToCore(mpuThreadFunction, "MPU6050 Thread", 10000, NULL, 0, &mpuThread, core);
}

void MPU6050_Class::update(uint8_t mpuData) {
    if(mpuData > 5) return;
    uint8_t msbRegister = ((mpuData < 3) ? 0x43 : 0x35) + (mpuData * 2);
    int16_t rawData = (((int8_t)readRegister(msbRegister)) << 8) | readRegister(msbRegister + 1);
    rawData -= calibrationValues[mpuData];
    if(mpuData < 3) {
        data[mpuData] += rawData * ((250.0 / 0x7fff) / sampleRate);
    }
    else {
        data[mpuData] += rawData * (((9.81 * 2) / 0x7fff) / sampleRate);
    }
}

void MPU6050_Class::calibrate(uint8_t mpuData, uint8_t sampleCount) {
    int32_t rawDataSum = 0;
    for(uint8_t i = 0; i < sampleCount; i++) {
        uint64_t lastMicros = 0;
        if(micros() - lastMicros >= 1000000 / sampleRate) {
            lastMicros = micros();
            uint8_t msbRegister = ((mpuData < 3) ? 0x43 : 0x35) + (mpuData * 2);
            rawDataSum += (((int16_t)readRegister(msbRegister)) << 8) | readRegister(msbRegister + 1);
        }
    }
    calibrationValues[mpuData] = (int16_t)(rawDataSum / sampleCount);
}

uint8_t MPU6050_Class::readRegister(uint8_t address) {
    _i2c->beginTransmission(_address);
    _i2c->write(address);
    _i2c->endTransmission(false);
    _i2c->requestFrom(_address, (uint8_t)1);
    while(_i2c->available() < 1) {}
    return (uint8_t)_i2c->read();
}

void MPU6050_Class::writeRegister(uint8_t address, uint8_t value) {
    _i2c->beginTransmission(_address);
    _i2c->write(address);
    _i2c->write(value);
    _i2c->endTransmission();
}

MPU6050_Class mpu6050;
