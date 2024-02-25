/**
 * WRO Main
 * by TerraForce
*/

#define WRO_MAIN_VERSION "1.0.0"


// oled display debug
#define OLED_DISPLAY

// serial debug
#define SERIAL_DEBUG

// serial debug features
//#define DEBUG_BATTERY_VOLTAGE
//#define DEBUG_MOTOR_TURNS
//#define DEBUG_ULTRASONIC
//#define DEBUG_SERVO_COMMANDS
//#define DEBUG_LED_COMMANDS
//#define DEBUG_I2C_SCAN
#define DEBUG_ROTATION

#define VOLTAGE_BATTERY_CHARGED     7.2
#define VOLTAGE_BATTERY_EMPTY       6.8

#pragma region includes

#include <Arduino.h>
#include <Wire.h>

#ifdef OLED_DISPLAY
    #include <Adafruit_SSD1306.h>
#endif

#ifdef SERIAL_DEBUG
    #include <HardwareSerial.h>
#endif



#pragma endregion includes


#pragma region pin_definitions

// pin of the start button
#define Pin_Start_Button            (uint8_t)8

// pins for I2C communication
#define Pin_I2C_MASTER_SDA          (uint8_t)16
#define Pin_I2C_MASTER_SCL          (uint8_t)17
#define Pin_I2C_SLAVE_SDA           (uint8_t)7
#define Pin_I2C_SLAVE_SCL           (uint8_t)15

// camera request interrupt pins
#define Pin_Rotation_Request        (uint8_t)5
#define Pin_Camera_Request          (uint8_t)6

enum UltraSonicPositions {
    US_LeftFront,
    US_CenterFront,
    US_RightFront,
    US_LeftBack,
    US_CenterBack,
    US_RightBack,
};

// trigger and echo pins of the ultrasonic sensors
#define Pins_UltraSonic_Trig        (uint8_t[]){9, 11, 13, 48, 37, 1}
#define Pins_UltraSonic_Echo        (uint8_t[]){10, 12, 14, 47, 36, 38}

#pragma endregion pin_definitions


#pragma region global_properties

struct SENSOR_DATA {
    uint16_t analogValues[4];
    uint32_t motorTurns[2];
} sensorData = {};
TwoWire i2c_master(0);
TwoWire i2c_slave(1);
TaskHandle_t ultrasonicThread;
uint32_t ultrasonicDistance[6] = {}; // distance to object in front of ultrasonic sensor in mm
int32_t rotation = 0; // rotation in 1/10 degrees

#ifdef OLED_DISPLAY
    Adafruit_SSD1306 oled(128, 32, &i2c_master, -1, 400000, 400000);
#endif

#ifdef SERIAL_DEBUG
    HardwareSerial loggingSerial(0);
#endif

enum Borders {
    Unknown,
    Left,
    Right
};

uint8_t outsideBorder = Unknown;
uint8_t curve = Unknown;
int32_t startRotation = 0; // rotation to curve target in 1/10°

#pragma endregion global_properties


#pragma region functions

void ultrasonicThreadFunction(void* parameter);
void i2cOnReceiveFunction(int bytes);
void setServo(uint8_t index, int8_t speed);
void setLight(uint8_t index, bool state);
void updateRotation();
void updateCameraData();
void updateVoltageAndRPM();

#pragma endregion functions


void setup() {
    
    #ifdef SERIAL_DEBUG
        // start serial (UART)
        loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);
        loggingSerial.println(String("\nWRO Main\nVersion: ") + WRO_MAIN_VERSION + "\n");
    #endif

    // start I2C as master in fast mode
    i2c_master.begin(Pin_I2C_MASTER_SDA, Pin_I2C_MASTER_SCL, 400000);

    // start I2C as slave in fast mode
    i2c_slave.begin(0x51, Pin_I2C_SLAVE_SDA, Pin_I2C_SLAVE_SCL, 400000);
    i2c_slave.onReceive(i2cOnReceiveFunction);

    #ifdef OLED_DISPLAY
        // start OLED
        oled.begin(SSD1306_SWITCHCAPVCC, 0x3c, true, false);
        oled.clearDisplay();
        oled.display();
    #endif

    // set pin mode of start button
    pinMode(Pin_Start_Button, INPUT);

    // set pin modes for ultrasonic sensors
    for(uint8_t i = 0; i < 6; i++) {
        pinMode(Pins_UltraSonic_Trig[i], OUTPUT);
        digitalWrite(Pins_UltraSonic_Trig[i], LOW);
        pinMode(Pins_UltraSonic_Echo[i], INPUT);
    }

    pinMode(Pin_Rotation_Request, INPUT);
    pinMode(Pin_Camera_Request, INPUT);
    digitalWrite(Pin_Rotation_Request, LOW);
    digitalWrite(Pin_Camera_Request, LOW);

    delay(2000);

    setLight(0, 1);
    setLight(1, 1);

    xTaskCreatePinnedToCore(ultrasonicThreadFunction, "Ultrasonic Thread", 10000, NULL, 0, &ultrasonicThread, 1 - xPortGetCoreID());
}

void loop() {

    updateVoltageAndRPM();
    updateRotation();

    #ifdef DEBUG_BATTERY_VOLTAGE
        loggingSerial.print("Battery Voltage: ");
        loggingSerial.print(sensorData.analogValues[0] * 0.009765625, 2);
        loggingSerial.println(" V");
    #endif

    #ifdef DEBUG_MOTOR_TURNS
        loggingSerial.print("Motor Turns: ");
        loggingSerial.println(sensorData.motorTurns[0], DEC);
    #endif

    #ifdef DEBUG_ROTATION
        loggingSerial.print("Rotation: ");
        loggingSerial.print(rotation / 10.0, 1);
        loggingSerial.println("°");
    #endif

    #ifdef OLED_DISPLAY
        oled.clearDisplay();
        oled.setCursor(50, 3);
        oled.setTextSize(1);
        oled.setTextColor(0xffff);
        oled.print(sensorData.analogValues[0] * 0.009765625, 2);
        oled.print(" V");
        oled.drawRect(100, 2, 26, 10, 0xffff);
        float chargeLevel = ((sensorData.analogValues[0] * 0.009765625) - VOLTAGE_BATTERY_EMPTY) / (VOLTAGE_BATTERY_CHARGED - VOLTAGE_BATTERY_EMPTY);
        oled.fillRect(100, 2, (uint16_t)(26 * chargeLevel), 10, 0xffff);
        oled.display();
    #endif

    /*i2c_master.requestFrom(0x51, 2);
    int16_t rotation = 0;
    while(i2c_master.available() < 2){
        delay(1);
    }
    i2c_master.readBytes((uint8_t*)(&rotation), 2);
    loggingSerial.println(rotation / 10.0, 1);
    delay(500);

    int32_t ultrasonicDistanceDifferenceLeft = ultrasonicDistance[US_LeftBack] - ultrasonicDistance[US_LeftFront];
    int32_t ultrasonicDistanceDifferenceRight = ultrasonicDistance[US_RightBack] - ultrasonicDistance[US_RightFront];

    if(curve != Unknown) {
        if(((curve == Left) && (rotation <= startRotation + 30)) || ((curve == Right) && (rotation >= startRotation - 30))){
            curve = Unknown;
            setServo(1, 0);
        }
    }
    else if(outsideBorder == Unknown) {
        if(ultrasonicDistanceDifferenceLeft <= -150) {
            outsideBorder = Right;
        }
        else if(ultrasonicDistanceDifferenceRight <= -150) {
            outsideBorder = Left;
        }
        else {
            setServo(0, 15);
        }
    }
    else {
        if(ultrasonicDistanceDifferenceLeft <= -150) {
            if(outsideBorder == Right) {
                setServo(1, -15);
                curve = Left;
                startRotation -= 900;
            }
            else {
                outsideBorder == Left;
            }
        }
        else if(ultrasonicDistanceDifferenceRight <= -150) {
            if(outsideBorder == Left) {
                setServo(1, 15);
                curve = Right;
                startRotation += 900;
            }
            else {
                outsideBorder == Right;
            }
        }
        else if(((rotation - startRotation) > 50) || (ultrasonicDistance[US_RightFront] <= 100)){
            setServo(1, -10);
            curve = Left;
        }
        else if(((rotation - startRotation) < 50) || (ultrasonicDistance[US_LeftFront] <= 100)){
            setServo(1, 10);
            curve = Right;
        }
        else {
            setServo(1, 0);
        }
    }*/

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

void ultrasonicThreadFunction(void* parameter) {
    while(true) {

        #ifdef DEBUG_ULTRASONIC
            int64_t startTime = esp_timer_get_time();
        #endif

        for(uint8_t i = 0; i < 6; i++) {
            digitalWrite(Pins_UltraSonic_Trig[i], HIGH);
            delay(10);
            digitalWrite(Pins_UltraSonic_Trig[i], LOW);
            ultrasonicDistance[i] = pulseIn(Pins_UltraSonic_Echo[i], HIGH) * 0.1716;
        }

        #ifdef DEBUG_ULTRASONIC
            loggingSerial.print("UltraSonic Rate: ");
            loggingSerial.print(1000000.0 / (esp_timer_get_time() - startTime), 2);
            loggingSerial.println(" hz");
            for(uint8_t i = 0; i < 6; i++) {
                loggingSerial.print(ultrasonicDistance[i]);
                loggingSerial.println(" mm");
            }
        #endif
    }
}

void i2cOnReceiveFunction(int bytes) {
    if(bytes == 4) {
        i2c_slave.readBytes((uint8_t*)&rotation, 4);
    }
    else {

    }
}

void setServo(uint8_t index, int8_t speed) {
    i2c_master.beginTransmission(0x50);
    i2c_master.write(((index & 0x3) << 5) | ((speed < 0) * 0x10) | (((speed < 0) ? (-speed) : speed)) & 0xf);
    i2c_master.endTransmission();
}

void setLight(uint8_t index, bool state) {
    i2c_master.beginTransmission(0x50);
    i2c_master.write(0x80 | ((index & 0x3f) << 1) | state);
    i2c_master.endTransmission();
}

void updateRotation() {
    digitalWrite(Pin_Rotation_Request, HIGH);
    delay(10);
    digitalWrite(Pin_Rotation_Request, LOW);
}

void updateCameraData() {
    digitalWrite(Pin_Camera_Request, HIGH);
    delay(10);
    digitalWrite(Pin_Camera_Request, LOW);
}

void updateVoltageAndRPM() {
    i2c_master.requestFrom(0x50, sizeof(SENSOR_DATA));
    while(i2c_master.available() < sizeof(SENSOR_DATA)) {
        delay(1);
    }
    i2c_master.readBytes((uint8_t*)&sensorData, sizeof(SENSOR_DATA));
}
