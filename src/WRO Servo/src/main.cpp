/**
 * WRO Servo
 * by Robert Flugrat
*/

#define WRO_SERVO_VERSION "1.0.0"

#define TEST_MODE

#pragma region includes

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <servo.h>

#pragma endregion includes


#pragma region pin_definitions

#define Pins_Interrupt              (uint8_t[]){2, 3}
#define Pins_LEDs                   (uint8_t[]){11, 10, 9, 8, 7, 6, 5, 4}
#define Pins_PWM                    (uint8_t[]){12, A1, A0, 13}
#define Pins_ADC                    (uint8_t[]){A2, A3, A6, A7}

#pragma endregion pin_definitions


#pragma region global_properties

TwoWire i2c = Wire;
SERVOS servo;
uint32_t motorTurns1 = 0;
uint32_t motorTurns2 = 0;
uint64_t motorTurnsStartMillis = 0;
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
    Serial.begin(115200);
    Serial.println("WRO Servo");
    Serial.println(String("Version ") + WRO_SERVO_VERSION);
    Serial.println();

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
    motorTurnsStartMillis = millis();

    i2c.onRequest(i2cOnRequestGuardFunction);
    i2c.onReceive(i2cOnReceiveFunction);
}

void loop() {}

void interruptFunction1() {
    motorTurns1++;
}

void interruptFunction2() {
    motorTurns2++;
}

void i2cOnRequestGuardFunction() {
    if(!i2cRequestWorking){
        i2cRequestWorking = true;
        i2cOnRequestFunction();
        i2cRequestWorking = false;
    }
}

void i2cOnRequestFunction() {
    Serial.println("I2C request");
    uint16_t buffer[6];
    for(uint8_t i = 0; i < 4; i++){
        buffer[i] = analogRead(Pins_ADC[i]);
    }
    buffer[4] = (uint16_t)((motorTurns1 * 100000.0) / motorTurnsStartMillis); // 0,01 turns per second
    buffer[5] = (uint16_t)((motorTurns2 * 100000.0) / motorTurnsStartMillis); // 0,01 turns per second
    i2c.write((char*)buffer, 12);
}

void i2cOnReceiveFunction(int bytes) {
    Serial.println("I2C receive");
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
