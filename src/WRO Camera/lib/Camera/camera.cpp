#include "camera.h"

CAMERA::RGB888ROW::RGB888ROW(RGB888** rowStart, uint16_t x, uint16_t width) {
    _rowStart = rowStart;
    _x = x;
    _width = width;
}

RGB888 CAMERA::RGB888ROW::operator[](uint16_t y) {
    return (*_rowStart)[_x + (y * _width)];
}

bool CAMERA::init() {
    frame = (RGB888*)ps_calloc(320 * 240, sizeof(RGB888));
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
        .frame_size = FRAMESIZE_QVGA,
        .jpeg_quality = 4,
        .fb_count = 1,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    };
    if(esp_camera_init(&camera_config) != ESP_OK) {
        return false;
    }
    _sensor = esp_camera_sensor_get();
    _settings = esp_camera_sensor_get_info(&_sensor->id);
    return true;
}

bool CAMERA::capture() {
    camera_fb_t* frameBuffer = esp_camera_fb_get();
    if(!frameBuffer) {
        return false;
    }
    if(!fmt2rgb888(frameBuffer->buf, frameBuffer->len, frameBuffer->format, (uint8_t*)frame)) {
        return false;
    }
    height = frameBuffer->height;
    width = frameBuffer->width;
    esp_camera_fb_return(frameBuffer);
    return true;
}

CAMERA::RGB888ROW CAMERA::operator[](uint16_t x) {
    return CAMERA::RGB888ROW(&frame, x, width);
}