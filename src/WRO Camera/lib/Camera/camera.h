#ifndef CAMERA_H
#define CAMERA_H

/**
 * Camera library for ESP32 Camera
 * by TerraForce
*/

#include <Arduino.h>
#include <esp_camera.h>
#include <FS.h>

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

struct RGB {
    uint16_t color;
    uint8_t r() {return (color & 0xF800) >> 8;}
    uint8_t g() {return (color & 0x07E0) >> 3;}
    uint8_t b() {return (color & 0x001F) << 3;}
};

enum FrameSize {
    FS_96X96,    // 96x96
    FS_QQVGA,    // 160x120
    FS_QCIF,     // 176x144
    FS_HQVGA,    // 240x176
    FS_240X240,  // 240x240
    FS_QVGA,     // 320x240
    FS_CIF,      // 400x296
    FS_HVGA,     // 480x320
    FS_VGA,      // 640x480
    FS_SVGA,     // 800x600
    FS_XGA,      // 1024x768
    FS_HD,       // 1280x720
    FS_SXGA,     // 1280x1024
    FS_UXGA,     // 1600x1200
};

#define FramePixels (uint32_t[]){ 9216, 19200, 25344, 42240, 57600, 76800, 118400, \
                    153600, 307200, 480000, 786432, 921600, 1310720, 1920000 }

class CAMERA {
    private:
        friend class RGBROW;
    public:
        // Friend Class
        class RGBROW {
            public:
                // Constructor
                RGBROW(uint8_t* rowStart, uint16_t x, uint16_t width);

                // Functions
                RGB operator[](uint16_t y);

            private:
                uint8_t* _rowStart = NULL;
                uint16_t _x = 0;
                uint16_t _width = 0;
        };

        // Functions
        bool init(FrameSize frameSize);
        bool capture();
        bool save(File* file);

        void setBrightness(uint8_t level);  // -2 - 2
        void setContrast(uint8_t level);    // -2 - 2
        void setSaturation(uint8_t level);  // -2 - 2
        void setSharpness(uint8_t level);   // -2 - 2

        RGBROW operator[](uint16_t x);

        // Properties
        uint16_t height = 0;
        uint16_t width = 0;

    private:
        sensor_t* sensor;
        camera_sensor_info_t* _settings;
        camera_fb_t* frameBuffer;
};

#endif