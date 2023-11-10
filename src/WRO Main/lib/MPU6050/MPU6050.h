#ifndef MPU6050_H
#define MPU6050_H

/**
 * MPU6050 library for ESP32 MCUs
 * by Robert Flugrat
*/

#define MPU6050_VERSION "1.0.0"

#include <Arduino.h>
#include <Wire.h>

template <typename type>
class TRIP {
public:
	type x;
	type y;
	type z;
};

class MPU6050 {

    //
    //
    void init(::TwoWire i2c);

    //
    //
    TRIP<uint16_t> angle;
};

#endif
