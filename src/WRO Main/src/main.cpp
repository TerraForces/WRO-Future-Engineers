/**
 * WRO Main
 * by TerraForce
*/

#define WRO_MAIN_VERSION "1.2.0"

// oled display debug
//#define OLED_DISPLAY

// serial debug
//#define SERIAL_DEBUG

// serial debug features#
//#define DEBUG_BATTERY_VOLTAGE
//#define DEBUG_MOTOR_TURNS
//#define DEBUG_ULTRASONIC
//#define DEBUG_SERVO_COMMANDS
//#define DEBUG_LED_COMMANDS
//#define DEBUG_I2C_SCAN
//#define DEBUG_ROTATION

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
#define Pin_Start_Button            (uint8_t) 8

// pins for I2C communication
#define Pin_I2C_MASTER_SDA          (uint8_t) 18
#define Pin_I2C_MASTER_SCL          (uint8_t) 17
#define Pin_I2C_SLAVE_SDA           (uint8_t) 7
#define Pin_I2C_SLAVE_SCL           (uint8_t) 15

enum UltraSonicPositions {
    US_LeftFront,
    US_CenterFront,
    US_RightFront,
    US_LeftBack,
    US_CenterBack,
    US_RightBack,
    Variable
};

// trigger and echo pins of the ultrasonic sensors
#define Pins_UltraSonic_Trig        (uint8_t[]){ 9, 11, 13, 48, 37, 1 }
#define Pins_UltraSonic_Echo        (uint8_t[]){ 10, 12, 14, 47, 36, 38 }

#define UltraSonic_Process          (uint8_t[]){US_LeftFront, US_CenterFront, US_RightFront, US_LeftBack, Variable, US_RightBack}

#pragma endregion pin_definitions


#pragma region global_properties

struct POWER_SENSOR_DATA {
    uint16_t analogValues[4]; // 10 bit resolution in 5V
    uint32_t motorTurns[2]; // 1/8 motor turns
} powerSensorData = {};

struct CAMERA_SENSOR_DATA {
    int32_t rotation; // rotation in 1/10 degrees
} cameraSensorData = {};

TaskHandle_t ultrasonicThread;
uint16_t ultrasonicDistance[6] = {}; // distance to object in front of ultrasonic sensor in mm
uint8_t ultrasonicProcess = 0;

int8_t servoState[4] = {};
uint8_t lightState = 0;

TwoWire i2c_master(0);
TwoWire i2c_slave(1);

#ifdef OLED_DISPLAY
    Adafruit_SSD1306 oled(128, 32, &i2c_master, -1, 400000, 400000);
#endif

#ifdef SERIAL_DEBUG
    HardwareSerial loggingSerial(0);
#endif

enum Directions {
    Straight,
    Left,
    Right,
    End
};

enum DriveStates {
    Unknown,
    Curve,
    CurveEnding,
    GyroCorrection,
    UltrasonicCorrection,
    BorderCorrection
};

struct DRIVE_STATE {
    uint8_t direction   : 2;
    uint8_t state       : 6;
} driveState = {};

uint8_t outsideBorder = Unknown;
int32_t rotation = 0, antiRotation = 0;
int32_t targetRotation = 0; // rotation to curve target in 1/10°
uint64_t lastCurve = 0;
uint8_t curveCount = 0;
uint16_t startPosDistance = 0;

#pragma endregion global_properties


#pragma region functions

void driveControl();
void fireUltrasonic(uint8_t num);
void ultrasonicThreadFunction(void* parameter);
void setServo(uint8_t index, int8_t speed);
void setLight(uint8_t index, bool state);
void i2cOnReceiveFunction(int bytes);
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

    #ifdef OLED_DISPLAY
        // start and clear OLED
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

    delay(2000);

    // start I2C as slave in fast mode
    i2c_slave.begin(0x51, Pin_I2C_SLAVE_SDA, Pin_I2C_SLAVE_SCL, 400000);
    i2c_slave.onReceive(i2cOnReceiveFunction);

    // switch on all lights
    setLight(0, 0);
    setLight(1, 0);
    
    // start ultrasonic sensor thread
    xTaskCreatePinnedToCore(ultrasonicThreadFunction, "Ultrasonic Thread", 10000, NULL, 0, &ultrasonicThread, 1 - xPortGetCoreID());

    #ifndef SERIAL_DEBUG
        // wait for start signal
        while(digitalRead(Pin_Start_Button)) {
            delay(1);
        }
        setServo(0, 7);
        antiRotation = rotation;
        startPosDistance = ultrasonicDistance[US_CenterFront];
    #endif
}

void loop() {

    //updateVoltageAndRPM();

    #ifndef SERIAL_DEBUG
        driveControl();
    #endif

    #ifdef DEBUG_BATTERY_VOLTAGE
        loggingSerial.print("Battery Voltage: ");
        loggingSerial.print(powerSensorData.analogValues[0] * 0.009765625, 2);
        loggingSerial.println(" V");
    #endif

    #ifdef DEBUG_MOTOR_TURNS
        loggingSerial.print("Motor Turns: ");
        loggingSerial.println(powerSensorData.motorTurns[0] / 8.0, 3);
    #endif

    #ifdef DEBUG_ROTATION
        loggingSerial.print("Rotation: ");
        loggingSerial.print(cameraSensorData.rotation / 10.0, 1);
        loggingSerial.println("°");
    #endif

    #ifdef OLED_DISPLAY
        oled.clearDisplay();
        oled.setCursor(50, 3);
        oled.setTextSize(1);
        oled.setTextColor(0xffff);
        oled.print(powerSensorData.analogValues[0] * 0.009765625, 2);
        oled.print(" V");
        oled.drawRect(100, 2, 26, 10, 0xffff);
        float chargeLevel = ((powerSensorData.analogValues[0] * 0.009765625) - VOLTAGE_BATTERY_EMPTY) / (VOLTAGE_BATTERY_CHARGED - VOLTAGE_BATTERY_EMPTY);
        oled.fillRect(100, 2, (uint16_t)(26 * chargeLevel), 10, 0xffff);
        oled.display();
    #endif

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
    
    #ifdef SERIAL_DEBUG
        delay(100);
    #else
        delay(15);
    #endif
}

void driveControl() {

    // detect inner and outer border
    if(outsideBorder == Unknown) {
        if(ultrasonicDistance[US_LeftFront] > 1100) {
            outsideBorder = Right;
        }
        if(ultrasonicDistance[US_RightFront] > 1100) {
            outsideBorder = Left;
        }
    }
    if(outsideBorder != Unknown) {
        if((driveState.state != Curve) && (driveState.state != CurveEnding)) {
            if(curveCount < 12) {
                // detect curves
                if((millis() > lastCurve + 2000) && (outsideBorder == Right) && (ultrasonicDistance[US_LeftFront] > 1000)) {
                    driveState.direction = Left;
                    driveState.state = Curve;
                    antiRotation = cameraSensorData.rotation - targetRotation;
                    targetRotation -= 900;
                    setServo(0, 9);
                    setServo(1, -15);
                    setLight(0, 0);
                    setLight(1, 0);
                    curveCount++;
                }
                if((millis() > lastCurve + 2000) && (outsideBorder == Left) && (ultrasonicDistance[US_RightFront] > 1000)) {
                    driveState.direction = Right;
                    driveState.state = Curve;
                    antiRotation = cameraSensorData.rotation - targetRotation;
                    targetRotation += 900;
                    setServo(0, 9);
                    setServo(1, 15);
                    setLight(0, 0);
                    setLight(1, 0);
                    curveCount++;
                }
                if(curveCount == 12) {
                    outsideBorder = End;
                }
            }
            else {
                // find end position
                if((ultrasonicDistance[US_CenterFront] != 0) && (ultrasonicDistance[US_CenterFront] < startPosDistance + 50)) {
                    setServo(0, 0);
                }
            }
        }

        // drive straight again after curve
        if((driveState.state == Curve) && (((driveState.direction == Left) && (rotation <= targetRotation - 30)) || ((driveState.direction == Right) && (rotation >= targetRotation + 30)))) {
            setServo(0, 7);
            setServo(1, 0);
            driveState.state = CurveEnding;
        }

        // reset curve status
        if((driveState.state == CurveEnding) && (((driveState.direction == Left) && (ultrasonicDistance[US_LeftBack] < 1200)) || ((driveState.direction == Right) && (ultrasonicDistance[US_RightBack] < 1200)))) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            lastCurve = millis();
        }
    }

    if((driveState.state != Curve) && (driveState.state != CurveEnding)) {

        // direction correction (ultrasonic)
        if((ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] > 20) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] < -20)) {
            driveState.direction = Left;
            driveState.state = UltrasonicCorrection;
            setServo(1, -3);
            setLight(0, 1);
        }
        else if((driveState.state == UltrasonicCorrection) && (driveState.direction == Left) && (ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] < 10) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] > -10)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
            setLight(0, 0);
        }
        else if((ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] < -20) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] > 20)) {
            driveState.direction = Right;
            driveState.state = UltrasonicCorrection;
            setServo(1, 3);
            setLight(1, 1);
        }
        else if((driveState.state == UltrasonicCorrection) && (driveState.direction == Right) && (ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] > -10) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] < 10)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
            setLight(1, 0);
        }

        // near border correction (ultrasonic)
        if((driveState.state == BorderCorrection) && (ultrasonicDistance[US_LeftFront] > 150) && (ultrasonicDistance[US_RightFront] > 150)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
            setLight(0, 0);
            setLight(1, 0);
        }
        if((ultrasonicDistance[US_LeftFront] < 120) && (ultrasonicDistance[US_LeftFront] != 0)) {
            driveState.direction = Right;
            driveState.state = BorderCorrection;
            setServo(1, 6);
            setLight(1, 1);
        }
        else if((ultrasonicDistance[US_RightFront] < 120) && (ultrasonicDistance[US_RightFront] != 0)) {
            driveState.direction = Left;
            driveState.state = BorderCorrection;
            setServo(1, -6);
            setLight(0, 1);
        }
    }
}

void fireUltrasonic(uint8_t num) {
    digitalWrite(Pins_UltraSonic_Trig[num], HIGH);
    delay(1);
    digitalWrite(Pins_UltraSonic_Trig[num], LOW);
    ultrasonicDistance[num] = (uint16_t)((pulseIn(Pins_UltraSonic_Echo[num], HIGH) * 0.1716) - (((num == US_LeftBack) || (num == US_RightBack)) * 17.5));
}

void ultrasonicThreadFunction(void* parameter) {
    while(true) {

        #ifdef DEBUG_ULTRASONIC
            int64_t startTime = esp_timer_get_time();
        #endif

        for(uint8_t i = 0; i < 6; i++) {
            fireUltrasonic((UltraSonic_Process[i] == Variable) ? ((outsideBorder == Left) ? US_RightFront : ((outsideBorder == Right) ? US_LeftFront : US_CenterFront)) : UltraSonic_Process[i]);
            delay(35);
        }

        #ifdef DEBUG_ULTRASONIC
            loggingSerial.print("UltraSonic Rate: ");
            loggingSerial.print(1000000.0 / (esp_timer_get_time() - startTime), 2);
            loggingSerial.println(" hz");
            for(uint8_t i = 0; i < 6; i++) {
                loggingSerial.print(ultrasonicDistance[i]);
                loggingSerial.println(" mm");
            }
            delay(1000);
        #endif
    }
}

void setServo(uint8_t index, int8_t speed) {
    if(servoState[index] != speed) {
        servoState[index] = speed;
        i2c_master.beginTransmission(0x50);
        i2c_master.write(((index & 0x3) << 5) | ((speed < 0) * 0x10) | (((speed < 0) ? (-speed) : speed)) & 0xf);
        i2c_master.endTransmission();

        #ifdef DEBUG_SERVO_COMMANDS
            loggingSerial.print("Servo ");
            loggingSerial.print(index, 10);
            loggingSerial.print(": ");
            loggingSerial.println(speed, 10);
        #endif
    }
}

void setLight(uint8_t index, bool state) {
    if((lightState & (1 << index)) >> index != state) {
        lightState ^= 1 << index;
        i2c_master.beginTransmission(0x50);
        i2c_master.write(0x80 | ((index & 0x3f) << 1) | state);
        i2c_master.endTransmission();

        #ifdef DEBUG_LED_COMMANDS
            loggingSerial.print("LED ");
            loggingSerial.print(index, 10);
            loggingSerial.print(": ");
            loggingSerial.println(state ? "ON" : "OFF");
        #endif
    }
}

void i2cOnReceiveFunction(int bytes) {
    i2c_slave.readBytes((uint8_t*)&cameraSensorData, sizeof(CAMERA_SENSOR_DATA));
    rotation = cameraSensorData.rotation - antiRotation;
}

void updateVoltageAndRPM() {
    i2c_master.requestFrom(0x50, sizeof(POWER_SENSOR_DATA));
    while(i2c_master.available() < sizeof(POWER_SENSOR_DATA)) {
        delay(1);
    }
    i2c_master.readBytes((uint8_t*)&powerSensorData, sizeof(POWER_SENSOR_DATA));
}
