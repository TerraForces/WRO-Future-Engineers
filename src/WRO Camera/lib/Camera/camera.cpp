#include "camera.h"

CAMERA::RGB565ROW::RGB565ROW(RGB565** rowStart, uint16_t x, uint16_t width) {
    _rowStart = rowStart;
    _x = x;
    _width = width;
}

RGB565 CAMERA::RGB565ROW::operator[](uint16_t y) {
    return (*_rowStart)[_x + (y * _width)];
}

bool CAMERA::init() {
    frame = (RGB565*)ps_calloc(320 * 240, sizeof(RGB565));
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
        .frame_size = FRAMESIZE_VGA,
        .jpeg_quality = 0,
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
    memcpy(frameBuffer->buf, frame, sizeof(frame));
    height = frameBuffer->height;
    width = frameBuffer->width;
    esp_camera_fb_return(frameBuffer);
    return true;
}

CAMERA::RGB565ROW CAMERA::operator[](uint16_t x) {
    return CAMERA::RGB565ROW(&frame, x, width);
}