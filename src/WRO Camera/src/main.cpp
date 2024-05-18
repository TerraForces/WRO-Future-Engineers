/**
 * WRO Camera
 * by TerraForce
*/

#define WRO_CAMERA_VERSION "1.2.0"

// serial debug
#define SERIAL_DEBUG

// serial debug features
//#define DEBUG_I2C_SCAN
//#define DEBUG_ROTATION

#define SAVE_IMAGE_SD_CARD


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

enum ObjectColors {
    Green,
    Red
};

enum ObjectDirections {
    Left,
    Right
};

struct CAMERA_SENSOR_DATA {
    int32_t rotation; // rotation in 1/10 degrees

    struct OBJECT_DATA {
        uint8_t available   : 1;
        uint8_t color       : 1;
        uint8_t direction   : 1;
        uint8_t angle       : 5;
    } object;

} cameraSensorData = {};

#ifndef SAVE_IMAGE_SD_CARD
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

void imageAnalysis();

#ifndef SAVE_IMAGE_SD_CARD
    void i2cSendData();
#endif

#pragma endregion functions


#pragma region setup

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

#pragma endregion setup


#pragma region loop

void loop() {

    camera.capture();
    imageAnalysis();
    delay(2000);

    #ifdef SAVE_IMAGE_SD_CARD
        /*if(!SD_MMC.exists("/esp-cam-images")) {
            SD_MMC.mkdir("/esp-cam-images");
        }
        while(SD_MMC.exists("/esp-cam-images/" + String(imageCount) + ".bmp")) {
            imageCount++;
        }
        File file = SD_MMC.open("/esp-cam-images/" + String(imageCount) + ".bmp", "w", true);
        camera.save(&file);
        delay(2000);*/
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

#pragma endregion loop


#pragma region functions

#define MAX2(a, b) ((a) > (b) ? (a) : (b))
#define MAX3(a, b, c) (MAX2(MAX2(a, b), c))
#define MIN2(a, b) ((a) < (b) ? (a) : (b))
#define MIN3(a, b, c) (MIN2(MIN2(a, b), c))

#define AVERAGE_BRIGHTNESS  150

#define BlackR 15
#define BlackG 15
#define BlackB 15

void imageAnalysis() {
	uint32_t averageR = 0, averageG = 0, averageB = 0;
	for (uint16_t x = 0; x < camera.width; x += 10) {
		for (uint16_t y = camera.height * 0.4; y < camera.height * 0.85; y += 10) {
			averageR += camera[x][y].r();
			averageG += camera[x][y].g();
			averageB += camera[x][y].b();
		}
	}
	averageR = (uint32_t)(averageR / (camera.width * camera.height * 0.0045));
	averageG = (uint32_t)(averageG / (camera.width * camera.height * 0.0045));
	averageB = (uint32_t)(averageB / (camera.width * camera.height * 0.0045));
	int16_t correctionR = averageR - AVERAGE_BRIGHTNESS;
	int16_t correctionG = averageG - AVERAGE_BRIGHTNESS;
	int16_t correctionB = averageB - AVERAGE_BRIGHTNESS;
	for (uint16_t x = 0; x < camera.width; x += 10) {
		for (uint16_t y = camera.height * 0.4; y < camera.height * 0.85; y += 10) {
			camera[x][y].r() = MAX2(camera[x][y].r() - correctionR, 0);
            camera[x][y].g() = MAX2(camera[x][y].g() - correctionG, 0);
            camera[x][y].b() = MAX2(camera[x][y].b() - correctionB, 0);
		}
	}

    if(!SD_MMC.exists("/esp-cam-images")) {
        SD_MMC.mkdir("/esp-cam-images");
    }
    while(SD_MMC.exists("/esp-cam-images/" + String(imageCount) + ".bmp")) {
        imageCount++;
    }
    File file = SD_MMC.open("/esp-cam-images/" + String(imageCount) + ".bmp", "w", true);

    BMP_HEADER bmpHeader = {};
    bmpHeader.bfSize = 54 + (uint32_t)(0.03 * camera.width * camera.height);
    bmpHeader.bfOffBits = 54;
    bmpHeader.biSize = 40;
    bmpHeader.biWidth = camera.width / 10;
	bmpHeader.biHeight = camera.height / 10;
    bmpHeader.biPlanes = 1;
    bmpHeader.biBitCount = 24;
    bmpHeader.biSizeImage = (uint32_t)(0.03 * camera.width * camera.height);
    uint16_t bfType = 0x4d42;
    file.write((uint8_t*)(&bfType), sizeof(uint16_t));
    file.write((uint8_t*)(&bmpHeader), sizeof(BMP_HEADER));
    for(int16_t y = camera.height - 10; y >= 0; y -= 10) {
        for(uint16_t x = 0; x < camera.width; x += 10) {
            file.write(camera[x][y].b());
            file.write(camera[x][y].g());
            file.write(camera[x][y].r());
        }
    }
    file.close();

    /*uint16_t PixelX = 0;
    uint16_t PixelY = camera.height * 0.75;

    cameraSensorData.object.available = false;
    cameraSensorData.object.color = 0;
	
    loggingSerial.println("Image Analysis started ...");
	while (((camera[camera.width / 2][PixelY].r() > BlackR) || (camera[camera.width / 2][PixelY].g() > BlackG) || (camera[camera.width / 2][PixelY].b() > BlackB) )&& (PixelY > 0)) {
		PixelX = camera.width / 2;
		while ((camera[PixelX][PixelY].r() > BlackR) && (camera[PixelX][PixelY].g() > BlackG) && (camera[PixelX][PixelY].b() > BlackB) && PixelX > 0) {
			if ((camera[PixelX][PixelY].r() > camera[PixelX][PixelY].g() * 1.35) && (camera[PixelX][PixelY].r() > camera[PixelX][PixelY].b() * 1.35) && camera[PixelX][PixelY].r() > 20) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Red;
                loggingSerial.println("red");
				goto ImageAnalysisEnd;
			}
			if ((camera[PixelX][PixelY].g() > camera[PixelX][PixelY].r() * 1.45) && (camera[PixelX][PixelY].g() > camera[PixelX][PixelY].b() * 1.45) && camera[PixelX][PixelY].g() > 20 && camera[PixelX][PixelY].g()<100) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Green;
				goto ImageAnalysisEnd;
			}
			PixelX -= 10;
		}
		PixelX = camera.width / 2;
		while ((camera[PixelX][PixelY].r() > BlackR) && (camera[PixelX][PixelY].g() > BlackG) && (camera[PixelX][PixelY].b() > BlackB) && PixelX < camera.width) {
			if ((camera[PixelX][PixelY].r() > camera[PixelX][PixelY].g() * 1.35) && (camera[PixelX][PixelY].r() > camera[PixelX][PixelY].b() * 1.35)&& camera[PixelX][PixelY].r()>20) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Red;
                loggingSerial.println("red");
				goto ImageAnalysisEnd;
			}
			if ((camera[PixelX][PixelY].g() > camera[PixelX][PixelY].r() * 1.35) && (camera[PixelX][PixelY].g() > camera[PixelX][PixelY].b() * 1.25) && camera[PixelX][PixelY].g() > 40 && camera[PixelX][PixelY].g() < 100) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Green;
				goto ImageAnalysisEnd;
			}
			PixelX += 10;
		}
		PixelY -= 10;
	}
	ImageAnalysisEnd:
    cameraSensorData.object.direction = PixelX > camera.width;
    loggingSerial.print("PixelX: ");
    loggingSerial.println(PixelX, 10);
    loggingSerial.print("PixelY: ");
    loggingSerial.println(PixelY, 10);
    
    loggingSerial.print("R: ");
    loggingSerial.println(camera[PixelX][PixelY].r(), 10);
    loggingSerial.print("G: ");
    loggingSerial.println(camera[PixelX][PixelY].g(), 10);
    loggingSerial.print("B: ");
    loggingSerial.println(camera[PixelX][PixelY].b(), 10);*/

    /*for(uint16_t x = camera.width / 2; ; x += 10) {
        for(uint16_t y = camera.height / 2; ; y += 10) {

        }
    }*/

    if(cameraSensorData.object.available) {
        if(cameraSensorData.object.color) {
            loggingSerial.print("Found red object at");
        }
        else {
            loggingSerial.print("Found green object at");
        }
        if(cameraSensorData.object.direction) {
            loggingSerial.println(" right\n");
        }
        else {
            loggingSerial.println(" left\n");
        }
    }
    else {
        loggingSerial.println("No object found\n");
    }
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

#pragma endregion functions
