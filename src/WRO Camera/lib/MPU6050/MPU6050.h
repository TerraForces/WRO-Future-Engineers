#ifndef MPU6050_H
#define MPU6050_H

/**
 * MPU6050 library for ESP32 MCUs
 * by Robert Flugrat
*/

#define MPU6050_VERSION "1.0.0"

#include <Arduino.h>
#include <Wire.h>

enum MPU_DATA {
    Rotation_X,
    Rotation_Y,
    Rotation_Z,
    Velocity_X,
    Velocity_Y,
    Velocity_Z
};

class MPU6050_Class {
public:
    void init(::TwoWire* i2c, uint8_t address, uint8_t dataToUpdate, bool core, uint16_t samplesPerSecond = 250, uint16_t calibrationSamples = 1000);
    void update(uint8_t mpuData);
    void calibrate(uint8_t mpuData, uint8_t sampleCount);

    double data[6] = {};
    uint8_t updateData = 0;
    uint16_t sampleRate = 0;

private:
    uint8_t readRegister(uint8_t address);
    void writeRegister(uint8_t address, uint8_t value);

    uint8_t _address = 0;
    TaskHandle_t mpuThread;
    TwoWire* _i2c;
    int16_t calibrationValues[6] = {};
};

extern MPU6050_Class mpu6050;

#endif
