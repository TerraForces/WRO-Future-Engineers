#include "camera.h"

enum COLORS {
    R,
    G,
    B
};

CAMERA::RGBROW::RGBPIXEL::RGBBYTE::RGBBYTE(uint16_t* address, uint8_t color) {
    _address = address;
    _color = color;
}

CAMERA::RGBROW::RGBPIXEL::RGBBYTE::operator uint8_t() {
    switch(_color) {
        case R:
            return (*_address) & 0xF8;
            break;
        case G:
            return (((*_address) & 0xE000) >> 11) | (((*_address) & 0x7) << 5);
            break;
        case B:
            return ((*_address) & 0x1F00) >> 5;
            break;
    }
    return 0;
}

void CAMERA::RGBROW::RGBPIXEL::RGBBYTE::operator=(uint8_t value) {
    switch(_color) {
        case R:
            (*_address) = ((*_address) & (0xFF07)) | (value & 0xF8);
            break;
        case G:
            (*_address) = ((*_address) & (0x1FF8)) | ((value & 0xE0) >> 5) | ((value & 0x1C) << 11);
            break;
        case B:
            (*_address) = ((*_address) & (0xE0FF)) | ((value & 0xF8) << 5);
            break;
    }
}

CAMERA::RGBROW::RGBPIXEL::RGBPIXEL(uint8_t* rowStart, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    _address = (uint16_t*)(rowStart + ((x + (width * ((height - 1) - y))) * 2));
}

CAMERA::RGBROW::RGBPIXEL::RGBBYTE CAMERA::RGBROW::RGBPIXEL::r() {
    return RGBBYTE(_address, R);
}

CAMERA::RGBROW::RGBPIXEL::RGBBYTE CAMERA::RGBROW::RGBPIXEL::g() {
    return RGBBYTE(_address, G);
}

CAMERA::RGBROW::RGBPIXEL::RGBBYTE CAMERA::RGBROW::RGBPIXEL::b() {
    return RGBBYTE(_address, B);
}

CAMERA::RGBROW::RGBROW(uint8_t* rowStart, uint16_t x, uint16_t width, uint16_t height) {
    _rowStart = rowStart;
    _x = x;
    _width = width;
    _height = height;
}

CAMERA::RGBROW::RGBPIXEL CAMERA::RGBROW::operator[](uint16_t y) {
    return CAMERA::RGBROW::RGBPIXEL(_rowStart, _x, y, _width, _height);
}

bool CAMERA::init(FrameSize frameSize) {
    camera_config_t camera_config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,
        .pixel_format = PIXFORMAT_RGB565,
        .frame_size = (framesize_t)(uint8_t)frameSize,
        .jpeg_quality = 0,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };
    if(esp_camera_init(&camera_config) != ESP_OK) {
        return false;
    }
    sensor = esp_camera_sensor_get();
    _settings = esp_camera_sensor_get_info(&sensor->id);
    sensor->set_hmirror(sensor, true);
    return true;
}

bool CAMERA::capture() {
    esp_camera_fb_return(frameBuffer);
    frameBuffer = esp_camera_fb_get();
    if(!frameBuffer) {
        return false;
    }
    height = frameBuffer->height;
    width = frameBuffer->width;
    return true;
}

bool CAMERA::save(File* file) {
    BMP_HEADER bmpHeader = {};
    bmpHeader.bfSize = 54 + (3 * width * height);
    bmpHeader.bfOffBits = 54;
    bmpHeader.biSize = 40;
    bmpHeader.biWidth = width;
	bmpHeader.biHeight = height;
    bmpHeader.biPlanes = 1;
    bmpHeader.biBitCount = 24;
    bmpHeader.biSizeImage = 3 * width * height;
    uint16_t bfType = 0x4d42;
    file->write((uint8_t*)(&bfType), sizeof(uint16_t));
    file->write((uint8_t*)(&bmpHeader), sizeof(BMP_HEADER));
    for(uint16_t lines = 0; lines < height; lines++) {
        uint8_t colorBuffer[width * 3];
        fmt2rgb888(frameBuffer->buf + (lines * width * 2), width * 2, PIXFORMAT_RGB565, colorBuffer);
        file->write(colorBuffer, sizeof(colorBuffer));
    }
    file->close();
    return true;
}

/*bool CAMERA::send(WiFiClient client) {
    client.print("HTTP/1.1 200 OK\r\nContent-Type: image/bmp\r\n\r\n");
    BMP_HEADER bmpHeader = {};
    bmpHeader.bfSize = 54 + (3 * width * height);
    bmpHeader.bfOffBits = 54;
    bmpHeader.biSize = 40;
    bmpHeader.biWidth = width;
	bmpHeader.biHeight = height;
    bmpHeader.biPlanes = 1;
    bmpHeader.biBitCount = 24;
    bmpHeader.biSizeImage = 3 * width * height;
    uint16_t bfType = 0x4d42;
    client.write((uint8_t*)(&bfType), sizeof(uint16_t));
    client.write((uint8_t*)(&bmpHeader), sizeof(BMP_HEADER));
    for(uint16_t lines = 0; lines < height; lines++) {
        uint8_t colorBuffer[width * 3];
        fmt2rgb888(frameBuffer->buf + (lines * width * 2), width * 2, PIXFORMAT_RGB565, colorBuffer);
        client.write(colorBuffer, sizeof(colorBuffer));
    }
    client.stop();
    return true;
}*/

void CAMERA::setBrightness  (uint8_t level) {sensor->set_brightness (sensor, level);}
void CAMERA::setContrast    (uint8_t level) {sensor->set_contrast   (sensor, level);}
void CAMERA::setSaturation  (uint8_t level) {sensor->set_saturation (sensor, level);}
void CAMERA::setSharpness   (uint8_t level) {sensor->set_sharpness  (sensor, level);}

CAMERA::RGBROW CAMERA::operator[](uint16_t x) {
    return CAMERA::RGBROW(frameBuffer->buf, x, width, height);
}
