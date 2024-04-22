/**
 * WRO Servo
 * by TerraForce
*/

#define WRO_SERVO_VERSION "1.1.0"


#pragma region includes

#include <Arduino.h>
#include <Wire.h>
#include <servo.h>

#pragma endregion includes


#pragma region pin_definitions

#define Pins_Interrupt              (uint8_t[]){ 2, 3 }
#define Pins_LEDs                   (uint8_t[]){ 11, 10, 9, 8, 7, 6, 5, 4 }
#define Pins_PWM                    (uint8_t[]){ 12, A1, A0, 13 }
#define Pins_ADC                    (uint8_t[]){ A2, A3, A6, A7 }

#pragma endregion pin_definitions


#pragma region global_properties

struct SENSOR_DATA {
    uint16_t analogValues[4];
    uint32_t motorTurns[2];
} sensorData = {};

TwoWire i2c = Wire;
SERVOS servo;
bool i2cRequestWorking = false;

#pragma endregion global_properties


#pragma region functions

void interruptFunction1();
void interruptFunction2();
void i2cOnRequestGuardFunction();
void i2cOnRequestFunction();
void i2cOnReceiveFunction(int bytes);

#pragma endregion functions

void setup() {

    i2c.setClock(400000);
    i2c.begin(0x50);

    for(uint8_t i = 0; i < 2; i++) {
        pinMode(Pins_Interrupt[i], INPUT);
    }

    for(uint8_t i = 0; i < 8; i++) {
        pinMode(Pins_LEDs[i], OUTPUT);
        digitalWrite(Pins_LEDs[i], LOW);
    }

    servo.init(4);
    for(uint8_t i = 0; i < 4; i++) {
        servo.set(Pins_PWM[i], 0);
    }

    for(uint8_t i = 0; i < 4; i++) {
        pinMode(Pins_ADC[i], INPUT);
    }

    attachInterrupt(digitalPinToInterrupt(Pins_Interrupt[0]), interruptFunction1, RISING);
    attachInterrupt(digitalPinToInterrupt(Pins_Interrupt[1]), interruptFunction2, RISING);

    i2c.onRequest(i2cOnRequestGuardFunction);
    i2c.onReceive(i2cOnReceiveFunction);
}

void loop() {}

void interruptFunction1() {
    sensorData.motorTurns[0]++;
}

void interruptFunction2() {
    sensorData.motorTurns[1]++;
}

void i2cOnRequestGuardFunction() {
    if(!i2cRequestWorking) {
        i2cRequestWorking = true;
        i2cOnRequestFunction();
        i2cRequestWorking = false;
    }
}

void i2cOnRequestFunction() {
    for(uint8_t i = 0; i < 4; i++) {
        sensorData.analogValues[i] = analogRead(Pins_ADC[i]);
    }
    i2c.write((uint8_t*)&sensorData, sizeof(SENSOR_DATA));
}

void i2cOnReceiveFunction(int bytes) {
    for(uint8_t i = 0; i < bytes; i++) {
        uint8_t command = i2c.read();
        if(command & 0b10000000) {
            digitalWrite(Pins_LEDs[(command & 0b1111110) >> 1], command & 0b1);
        }
        else {
            servo.set(Pins_PWM[(command & 0b1100000) >> 5], command & 0b10000 ? ((command & 0b1111) * 6) : -((command & 0b1111) * 6));
        }
    }
}
