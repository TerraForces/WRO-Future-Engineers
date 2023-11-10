#pragma region includes

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "camera.h"

#pragma endregion includes


#pragma region pin_definitions

// pins for I2C communication
#define Pin_I2C_SDA                 (uint8_t)12
#define Pin_I2C_SCL                 (uint8_t)13

#pragma endregion pin_definitions


#pragma region global_properties

CAMERA camera;
HardwareSerial loggingSerial(0);
TwoWire i2c(0);

#pragma endregion global_properties


#pragma region functions

#pragma endregion functions


void setup() {

    // start serial (UART) communication with PC for logging
    loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);
    loggingSerial.println("WRO Camera");

    // start I2C as slave in fast mode
    i2c.begin(0x51, Pin_I2C_SDA, Pin_I2C_SCL, 400000);

    psramInit();
    camera.init();
}

void loop() {

}
