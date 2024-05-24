/**
 * WRO Camera
 * by TerraForce
*/

#define WRO_CAMERA_VERSION "1.3.0"

// serial debug
// #define SERIAL_DEBUG

// serial debug features
// #define DEBUG_I2C_SCAN
// #define DEBUG_ROTATION

// saves image (after correction) on sd card
// #define SAVE_IMAGE_SD_CARD


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
        uint16_t available  : 1;
        uint16_t color      : 1;
        uint16_t direction  : 1;
        uint16_t x          : 7;
        uint16_t y          : 6;
    } object;

} cameraSensorData = {};

#ifndef SAVE_IMAGE_SD_CARD
    TwoWire i2c_master(0);
    bool interruptWorking = false;
#else
    uint32_t ImageCount = 0;
#endif

CAMERA camera;

#ifdef SERIAL_DEBUG
    HardwareSerial loggingSerial(0);
#endif

#pragma endregion global_properties


#pragma region functions

void ImageAnalysis();

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

    #ifndef SAVE_IMAGE_SD_CARD
        i2cSendData();
    #endif

    ImageAnalysis();

    #ifndef SAVE_IMAGE_SD_CARD
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
    #endif
}

#pragma endregion loop


#pragma region functions

#define MAX2(a, b) ((a) > (b) ? (a) : (b))
#define MAX3(a, b, c) (MAX2(MAX2(a, b), c))
#define MIN2(a, b) ((a) < (b) ? (a) : (b))
#define MIN3(a, b, c) (MIN2(MIN2(a, b), c))

// image processing parameters
#define Image_Upper_Height          0.4
#define Image_Lower_Height          0.8
#define Image_Density_Horizontal    10
#define Image_Density_Vertical      10

// image correction parameters
#define Image_Average_Brightness                150
#define Image_Brightness_Correction_Strength    1.0
#define Image_Color_Correction_Strength         0.5

// image analysis parameters
#define Image_Black_Value_R         40
#define Image_Black_Value_G         40
#define Image_Black_Value_B         40
#define Image_Min_Red_Value         60
#define Image_Min_Green_Value       40
#define Image_Max_Green_Value       200
#define Image_Red_Ratio             1.4
#define Image_Green_Ratio           1.4

void ImageAnalysis() {
	uint32_t averageR = 0, averageG = 0, averageB = 0;
	for (uint16_t x = 0; x < camera.width; x += Image_Density_Horizontal) {
		for (uint16_t y = camera.height * Image_Upper_Height; y < camera.height * Image_Lower_Height; y += Image_Density_Vertical) {
			averageR += camera[x][y].r();
			averageG += camera[x][y].g();
			averageB += camera[x][y].b();
		}
	}
    uint32_t pixelCount = (uint32_t)(camera.width * camera.height * (Image_Lower_Height - Image_Upper_Height) / (Image_Density_Horizontal * Image_Density_Vertical));
	averageR = (uint32_t)(averageR / (double)pixelCount);
	averageG = (uint32_t)(averageG / (double)pixelCount);
	averageB = (uint32_t)(averageB / (double)pixelCount);
    uint16_t averageMin = MIN3(averageR, averageG, averageB);
	int16_t correctionR = (int16_t)(((averageR - averageMin) * Image_Color_Correction_Strength) + ((averageMin - Image_Average_Brightness) * Image_Brightness_Correction_Strength));
	int16_t correctionG = (int16_t)(((averageG - averageMin) * Image_Color_Correction_Strength) + ((averageMin - Image_Average_Brightness) * Image_Brightness_Correction_Strength));
    int16_t correctionB = (int16_t)(((averageB - averageMin) * Image_Color_Correction_Strength) + ((averageMin - Image_Average_Brightness) * Image_Brightness_Correction_Strength));
	for (uint16_t x = 0; x < camera.width; x += Image_Density_Horizontal) {
		for (uint16_t y = camera.height * Image_Upper_Height; y < camera.height * Image_Lower_Height; y += Image_Density_Vertical) {
			camera[x][y].r() = MIN2(MAX2(camera[x][y].r() - correctionR, 0), 255);
            camera[x][y].g() = MIN2(MAX2(camera[x][y].g() - correctionG, 0), 255);
            camera[x][y].b() = MIN2(MAX2(camera[x][y].b() - correctionB, 0), 255);
		}
	}

    #ifdef SAVE_IMAGE_SD_CARD
        if(!SD_MMC.exists("/esp-cam-images")) {
            SD_MMC.mkdir("/esp-cam-images");
        }
        while(SD_MMC.exists("/esp-cam-images/" + String(ImageCount) + ".bmp")) {
            ImageCount++;
        }
        File file = SD_MMC.open("/esp-cam-images/" + String(ImageCount) + ".bmp", "w", true);
        BMP_HEADER bmpHeader = {};
        bmpHeader.bfSize = 54 + (uint32_t)(pixelCount * 3);
        bmpHeader.bfOffBits = 54;
        bmpHeader.biSize = 40;
        bmpHeader.biWidth = camera.width / Image_Density_Horizontal;
        bmpHeader.biHeight = camera.height * (Image_Lower_Height - Image_Upper_Height) / Image_Density_Vertical;
        bmpHeader.biPlanes = 1;
        bmpHeader.biBitCount = 24;
        bmpHeader.biSizeImage = (uint32_t)(pixelCount * 3);
        uint16_t bfType = 0x4d42;
        file.write((uint8_t*)(&bfType), sizeof(uint16_t));
        file.write((uint8_t*)(&bmpHeader), sizeof(BMP_HEADER));
        for(int16_t y = (camera.height * Image_Lower_Height) - Image_Density_Vertical; y >= camera.height * Image_Upper_Height; y -= Image_Density_Vertical) {
            for(uint16_t x = 0; x < camera.width; x += Image_Density_Horizontal) {
                file.write(camera[x][y].b());
                file.write(camera[x][y].g());
                file.write(camera[x][y].r());
            }
        }
        file.close();
    #endif

    uint16_t PixelX = 0;
    uint16_t PixelY = camera.height * Image_Lower_Height;

    cameraSensorData.object.available = false;
    cameraSensorData.object.color = 0;
	
	while (((camera[camera.width / 2][PixelY].r() > Image_Black_Value_R) || (camera[camera.width / 2][PixelY].g() > Image_Black_Value_G) || (camera[camera.width / 2][PixelY].b() > Image_Black_Value_B)) && (PixelY >= camera.height * Image_Upper_Height)) {
		PixelX = camera.width / 2;
		while ((camera[PixelX][PixelY].r() > Image_Black_Value_R) && (camera[PixelX][PixelY].g() > Image_Black_Value_G) && (camera[PixelX][PixelY].b() > Image_Black_Value_B) && PixelX > 200) {
			if ((camera[PixelX][PixelY].r() > camera[PixelX][PixelY].g() * Image_Red_Ratio) && (camera[PixelX][PixelY].r() > camera[PixelX][PixelY].b() * Image_Red_Ratio) && (camera[PixelX][PixelY].r() > Image_Min_Red_Value)) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Red;
				goto ImageAnalysisEnd;
			}
			if ((camera[PixelX][PixelY].g() > camera[PixelX][PixelY].r() + 30) && (camera[PixelX][PixelY].g() < camera[PixelX][PixelY].r() + 120) && (camera[PixelX][PixelY].g() > camera[PixelX][PixelY].b() * Image_Green_Ratio) && (camera[PixelX][PixelY].g() > Image_Min_Green_Value)) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Green;
				goto ImageAnalysisEnd;
			}
			PixelX -= Image_Density_Horizontal;
		}
		PixelX = camera.width / 2;
		while ((camera[PixelX][PixelY].r() > Image_Black_Value_R) && (camera[PixelX][PixelY].g() > Image_Black_Value_G) && (camera[PixelX][PixelY].b() > Image_Black_Value_B) && (PixelX < camera.width -200)) {
			if ((camera[PixelX][PixelY].r() > camera[PixelX][PixelY].g() * Image_Red_Ratio) && (camera[PixelX][PixelY].r() > camera[PixelX][PixelY].b() * Image_Red_Ratio) && (camera[PixelX][PixelY].r() > Image_Min_Red_Value)) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Red;
				goto ImageAnalysisEnd;
			}
			if ((camera[PixelX][PixelY].g() > camera[PixelX][PixelY].r() + 30) && (camera[PixelX][PixelY].g() < camera[PixelX][PixelY].r() + 120) && (camera[PixelX][PixelY].g() > camera[PixelX][PixelY].b() * Image_Green_Ratio) && (camera[PixelX][PixelY].g() > Image_Min_Green_Value)) {
				cameraSensorData.object.available = true;
				cameraSensorData.object.color = Green;
				goto ImageAnalysisEnd;
			}
			PixelX += Image_Density_Horizontal;
		}
		PixelY -= Image_Density_Vertical;
	}
	ImageAnalysisEnd:
    cameraSensorData.object.direction = PixelX > camera.width / 2;
    cameraSensorData.object.x = (uint16_t)((PixelX / ((double)camera.width)) * 0x7F);
    cameraSensorData.object.y = (uint16_t)((PixelY / ((double)camera.height)) * 0x3F);
    
    #ifdef SERIAL_DEBUG
        loggingSerial.print("Target pixel:\nx: ");
        loggingSerial.println(PixelX);
        loggingSerial.print("y: ");
        loggingSerial.println(PixelY);
        loggingSerial.print("r: ");
        loggingSerial.println(camera[PixelX][PixelY].r());
        loggingSerial.print("g: ");
        loggingSerial.println(camera[PixelX][PixelY].g());
        loggingSerial.print("b: ");
        loggingSerial.println(camera[PixelX][PixelY].b());

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

#pragma endregion functions
