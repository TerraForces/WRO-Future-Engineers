/**
 * WRO Main
 * by TerraForce
*/

#define WRO_SERVO_VERSION "1.0.0"

// EN: enables test mode
// DE: Aktiviert den Testmodus
#define TEST_MODE

#pragma region includes

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>

#pragma endregion includes


#pragma region pin_definitions

// pin of the start button
#define Pin_Start_Button            (uint8_t)13

// pins for I2C communication
#define Pin_I2C_SDA                 (uint8_t)18
#define Pin_I2C_SCL                 (uint8_t)19

// pins of RGB LED
#define Pin_LED_R                   (uint8_t)21
#define Pin_LED_G                   (uint8_t)22
#define Pin_LED_B                   (uint8_t)23

enum UltraSonicPositions{
    US_LeftFront,
    US_CenterFront,
    US_RightFront,
    US_LeftBack,
    US_CenterBack,
    US_RightBack,
};

// trigger and echo pins of the ultrasonic sensors
#define Pins_UltraSonic_Trig        (uint8_t[]){4, 16, 17, 27, 26, 25}
#define Pins_UltraSonic_Echo        (uint8_t[]){33, 32, 35, 34, 39, 36}

#pragma endregion pin_definitions


#pragma region global_properties

// EN: I2C communication class
// DE: I2C-Kommunikationsklasse
TwoWire i2c(0);

TaskHandle_t ultrasonicThread;
uint32_t ultrasonicDistance[6] = {}; // distance to object in front of ultrasonic sensor in mm
HardwareSerial loggingSerial(0);

#pragma endregion global_properties


#pragma region functions

void ultrasonicThreadFunction(void* parameter);
void updateRGB(uint16_t analogValue);

#pragma endregion functions


void setup() {
    
    // EN: start serial (UART) communication with PC for logging at 115200 bit per second
    // DE: Starten der seriellen (UART) Kommunikation mit einem PC zur Protokollierung mit 115200 Bit pro Sekunde
    loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);

    // EN: print program name and version to serial port
    // DE: Ausgabe des Programmnamens und der Programmversion auf dem seriellen Anschluss
    loggingSerial.println("WRO Servo");
    loggingSerial.println(String("Version ") + WRO_SERVO_VERSION);
    loggingSerial.println();

    // start I2C as master in fast mode
    i2c.begin(Pin_I2C_SDA, Pin_I2C_SCL, 400000);

    i2c.setTimeOut(16000);

    // set pin mode of start button
    pinMode(Pin_Start_Button, INPUT_PULLUP);

    // set pin Modes of RGB LED
    pinMode(Pin_LED_R, OUTPUT);
    pinMode(Pin_LED_G, OUTPUT);
    pinMode(Pin_LED_B, OUTPUT);
    digitalWrite(Pin_LED_R, LOW);
    digitalWrite(Pin_LED_G, LOW);
    digitalWrite(Pin_LED_B, LOW);

    // set pin modes for ultrasonic sensors
    for(uint8_t i = 0; i < 6; i++) {
        pinMode(Pins_UltraSonic_Trig[i], OUTPUT);
        digitalWrite(Pins_UltraSonic_Trig[i], LOW);
        pinMode(Pins_UltraSonic_Echo[i], INPUT);
    }

    // create thread for requesting ultrasonic sensors on other core
    uint8_t currentCore = xPortGetCoreID();
    xTaskCreatePinnedToCore(ultrasonicThreadFunction, "Ultrasonic Thread", 10000, NULL, 0, &ultrasonicThread, 1 - currentCore);
}

void loop() {
    i2c.requestFrom(0x50, 12);
    uint16_t buffer[6];
    i2c.readBytes((char*)buffer, 12);
    updateRGB(buffer[0]);
    for(uint8_t i = 0; i < 6; i++){
        loggingSerial.println(buffer[i]);
    }
    delay(1000);
}

void ultrasonicThreadFunction(void* parameter) {
    while(true) {
        for(uint8_t i = 0; i < 6; i++) {
            // trigger ultrosonic
            digitalWrite(Pins_UltraSonic_Trig[i], HIGH);
            delayMicroseconds(20);
            digitalWrite(Pins_UltraSonic_Trig[i], LOW);
            uint32_t ultrasonicTime = pulseIn(Pins_UltraSonic_Echo[i], HIGH);
            ultrasonicDistance[i] = ultrasonicTime * 0.1716;
            delay(5);
        }
    }
}

void updateRGB(uint16_t analogValue) {
    float voltage = (10 / 1024) * analogValue;
    if(voltage < 7) {
        digitalWrite(Pin_LED_R, HIGH);
        digitalWrite(Pin_LED_G, LOW);
        digitalWrite(Pin_LED_B, LOW);
    }
    else if(voltage < 7.5) {
        digitalWrite(Pin_LED_R, HIGH);
        digitalWrite(Pin_LED_G, HIGH);
        digitalWrite(Pin_LED_B, LOW);
    }
    else {
        digitalWrite(Pin_LED_R, LOW);
        digitalWrite(Pin_LED_G, HIGH);
        digitalWrite(Pin_LED_B, LOW);
    }
}
