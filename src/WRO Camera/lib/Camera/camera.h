#ifndef CAMERA_H
#define CAMERA_H

/**
 * MPU6050 library for ESP32 Camera
 * by TerraForce
*/

#include <Arduino.h>
#include <esp_camera.h>

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

struct RGB565 {
    uint16_t r : 5;
    uint16_t g : 6;
    uint16_t b : 5;
};

class CAMERA {
    private:
        friend class RGB565ROW;
    public:
        // Friend Class
        class RGB565ROW {
            public:
                // Constructor
                RGB565ROW(RGB565** rowStart, uint16_t x, uint16_t width);

                // Functions
                RGB565 operator[](uint16_t y);

            private:
                RGB565** _rowStart = NULL;
                uint16_t _x = 0;
                uint16_t _width = 0;
        };

        // Functions
        bool init();
        bool capture();
        RGB565ROW operator[](uint16_t x);

        // Properties
        uint16_t height = 0;
        uint16_t width = 0;

    private:
        RGB565* frame;
        sensor_t* _sensor;
        camera_sensor_info_t* _settings;
};

#endif