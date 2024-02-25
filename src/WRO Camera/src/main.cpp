/**
 * WRO Camera
 * by TerraForce
*/

#define WRO_CAMERA_VERSION "1.0.0"

// serial debug
//#define SERIAL_DEBUG

// serial debug features
//#define DEBUG_I2C_SCAN
//#define DEBUG_ROTATION


#pragma region includes

#include <Arduino.h>
#include <Wire.h>
#include "camera.h"
#include "MPU6050.h"

#ifdef SERIAL_DEBUG
    #include <HardwareSerial.h>
#endif

#pragma endregion includes


#pragma region pin_definitions

// pins for I2C communication
#define Pin_I2C_MASTER_SDA          (uint8_t)2
#define Pin_I2C_MASTER_SCL          (uint8_t)14

// data request interrupt pins
#define Pin_Rotation_Request        (uint8_t)12
#define Pin_Camera_Request          (uint8_t)13

#pragma endregion pin_definitions


#pragma region global_properties

CAMERA camera;
TwoWire i2c_master(0);
bool i2cRequestWorking = false;

#ifdef SERIAL_DEBUG
    HardwareSerial loggingSerial(0);
#endif

#pragma endregion global_properties


#pragma region functions

void rotationRequestInterruptFunction();
void cameraRequestInterruptFunction();

#pragma endregion functions


void setup() {

    #ifdef SERIAL_DEBUG
        // start serial (UART)
        loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);
        loggingSerial.println(String("\nWRO Camera\nVersion: ") + WRO_CAMERA_VERSION + "\n");
    #endif

    // start I2C as master in fast mode
    i2c_master.begin(Pin_I2C_MASTER_SDA, Pin_I2C_MASTER_SCL, 400000);

    // disable onboard LED
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // start camera and PSRAM
    psramInit();
    camera.init();

    // start MPU6050
    mpu6050.init(&i2c_master, 0x69, (uint8_t)(1 << Rotation_Z), 1 - xPortGetCoreID());

    // enable interrupts for data requests
    pinMode(Pin_Rotation_Request, INPUT);
    pinMode(Pin_Camera_Request, INPUT);
    attachInterrupt(Pin_Rotation_Request, rotationRequestInterruptFunction, RISING);
    attachInterrupt(Pin_Camera_Request, cameraRequestInterruptFunction, RISING);
}

void loop() {

    //camera.capture();

    #ifdef DEBUG_ROTATION
        loggingSerial.println(mpu6050.data[Rotation_Z], 2);
    #endif

    #ifdef DEBUG_I2C_SCAN
        loggingSerial.println("Scanning for I2C devices ...");
        for(uint8_t i = 1; i < 0x7f; i++) {
            i2c_master.beginTransmission(i);
            if(i2c_master.endTransmission() == 0) {
                loggingSerial.print("I2C device found on address ");
                loggingSerial.println(i, HEX);
            }
        }
        loggingSerial.println("done.\n");
    #endif

    delay(500);
}

void rotationRequestInterruptFunction() {
    int32_t rotation = (int32_t)(mpu6050.data[Rotation_Z] * 10);
    i2c_master.beginTransmission(0x51);
    i2c_master.write((uint8_t*)&rotation, 4);
    i2c_master.endTransmission();
}

void cameraRequestInterruptFunction() {

}
