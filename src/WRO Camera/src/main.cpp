/**
 * WRO Camera
 * by TerraForce
*/

#define WRO_CAMERA_VERSION "1.1.0"

// serial debug
//#define SERIAL_DEBUG

// serial debug features
//#define DEBUG_I2C_SCAN
//#define DEBUG_ROTATION

//#define SAVE_IMAGE_SD_CARD
#define IMAGE_OVER_WIFI


#pragma region includes

#include <Arduino.h>
#include "camera.h"

#ifdef SERIAL_DEBUG
    #include <HardwareSerial.h>
#endif

#ifdef SAVE_IMAGE_SD_CARD
    #include <SD_MMC.h>
#else
    #include <Wire.h>
    #include "MPU6050.h"
#endif

#pragma endregion includes


#pragma region pin_definitions

#ifndef SAVE_IMAGE_SD_CARD
    // pins for I2C communication
    #define Pin_I2C_MASTER_SDA          (uint8_t) 14
    #define Pin_I2C_MASTER_SCL          (uint8_t) 13
#endif

#pragma endregion pin_definitions


#pragma region global_properties

#ifndef SAVE_IMAGE_SD_CARD
    struct CAMERA_SENSOR_DATA {
        int32_t rotation; // rotation in 1/10 degrees
    } cameraSensorData = {};
    TwoWire i2c_master(0);
    bool interruptWorking = false;
#else
    uint32_t imageCount = 0;
#endif

CAMERA camera;

#ifdef SERIAL_DEBUG
    HardwareSerial loggingSerial(0);
#endif

#pragma endregion global_properties


#pragma region functions

#ifndef SAVE_IMAGE_SD_CARD
    void i2cSendData();
#endif

#pragma endregion functions


void setup() {

    #ifdef SERIAL_DEBUG
        // start serial (UART)
        loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);
        loggingSerial.println(String("\nWRO Camera\nVersion: ") + WRO_CAMERA_VERSION + "\n");
    #endif

    #ifndef SAVE_IMAGE_SD_CARD
        // start I2C as master in fast mode
        i2c_master.begin(Pin_I2C_MASTER_SDA, Pin_I2C_MASTER_SCL, 400000);
    #endif

    // disable onboard LED
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // start camera and PSRAM
    psramInit();
    camera.init(FS_UXGA);

    #ifdef SAVE_IMAGE_SD_CARD
        //start SD
        SD_MMC.begin();
    #endif

    #ifndef SAVE_IMAGE_SD_CARD
        // start MPU6050
        mpu6050.init(&i2c_master, 0x68, (uint8_t)(1 << Rotation_Z), 1 - xPortGetCoreID());
    #endif
}

void loop() {

    camera.capture();

    #ifdef SAVE_IMAGE_SD_CARD
        if(!SD_MMC.exists("/esp-cam-images")) {
            SD_MMC.mkdir("/esp-cam-images");
        }
        while(SD_MMC.exists("/esp-cam-images/" + String(imageCount) + ".bmp")) {
            imageCount++;
        }
        File file = SD_MMC.open("/esp-cam-images/" + String(imageCount) + ".bmp", "w", true);
        camera.save(&file);
        digitalWrite(4, HIGH);
        delay(500);
        digitalWrite(4, LOW);
        delay(10000);
    #else
        i2cSendData();

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

        delay(10);
    #endif
}

#ifndef SAVE_IMAGE_SD_CARD
    void i2cSendData() {
        cameraSensorData.rotation = (int32_t)(mpu6050.data[Rotation_Z] * (-10.0));
        i2c_master.beginTransmission(0x51);
        i2c_master.write((uint8_t*)&cameraSensorData, sizeof(CAMERA_SENSOR_DATA));
        i2c_master.endTransmission();
        
        #ifdef DEBUG_ROTATION
            loggingSerial.println(cameraSensorData.rotation / 10.0, 1);
        #endif
    }
#endif
