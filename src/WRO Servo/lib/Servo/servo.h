#ifndef SERVO_H
#define SERVO_H

/**
 * Servo library for ATMega328 MCUs
 * by TerraForce
*/

#define SERVO_LIB_VERSION "1.0.0"

#include <Arduino.h>

#define SERVO_ANGLE_TO_256CLK(angle) (((uint8_t)round((1500 + (angle * (1000.0 / 180))) / 16.0)) - 1)

struct SERVORANGE {
    int16_t min;
    int16_t max;
};

struct SERVO {
    uint8_t pin;
    uint8_t delay;
    SERVORANGE range;
};

struct SERVODATA {
    SERVO* servos = NULL;
    uint8_t initCount = 0;
} static servoData;

class SERVOS {
    public:
        void init(const uint8_t count);
        void set(uint8_t pin, int16_t value, SERVORANGE range = {-90, 90});

        uint8_t maxCount = 0;
};

#endif
