/**
 * WRO Main
 * by TerraForce
*/

#define WRO_MAIN_VERSION "1.3.0"

// disable i2c features
#define DISABLE_LIGHT_COMMANDS

#define VOLTAGE_BATTERY_CHARGED     8.5
#define VOLTAGE_BATTERY_EMPTY       7.5
#pragma region includes

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>

#pragma endregion includes


#pragma region pin_definitions

// pins of the start button
#define Pin_Start_Button            (uint8_t) 8
#define Pin_Start_Button_LED        (uint8_t) 6

// pins for I2C communication
#define Pin_I2C_MASTER_SDA          (uint8_t) 18
#define Pin_I2C_MASTER_SCL          (uint8_t) 17
#define Pin_I2C_SLAVE_SDA           (uint8_t) 7
#define Pin_I2C_SLAVE_SCL           (uint8_t) 15

// TOF sensor shutdown pin
#define Pin_TOF_Shutdown            (uint8_t) 5

// program switch
#define Pin_Obstacle_Switch         (uint8_t) 4
#define Pin_Test_Mode_Switch        (uint8_t) 21

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

    struct OBJECT_DATA {
        uint8_t available   : 1;
        uint8_t color       : 1;
        uint8_t direction   : 1;
        uint8_t angle       : 5;
    } object;

} cameraSensorData = {};

TaskHandle_t ultrasonicThread;
uint16_t ultrasonicDistance[6] = {}; // distance to object in front of ultrasonic sensor in mm
uint8_t ultrasonicProcess = 0;

int8_t servoState[4] = {};
uint8_t lightState = 0;

TwoWire i2c_master(0);
TwoWire i2c_slave(1);

Adafruit_SSD1306 oled(128, 32, &i2c_master, -1, 400000, 400000);

HardwareSerial loggingSerial(0);

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
uint64_t lastCurve = 0, lastDisplayUpdate = 0;
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
void testAlgorithm();

#pragma endregion functions


#pragma region setup

void setup() {
    
    loggingSerial.setTxBufferSize(256);
    loggingSerial.begin(115200, SERIAL_8N1, -1, -1, false, 20000, 112);
    loggingSerial.println(String("\nWRO Main\nVersion: ") + WRO_MAIN_VERSION + "\n");

    // start I2C as master in fast mode
    if(i2c_master.begin(Pin_I2C_MASTER_SDA, Pin_I2C_MASTER_SCL, 400000)) {
        loggingSerial.println("SUCCESS - I2C master started");
    }
    else {
        loggingSerial.println("FAILED - I2C master start failed");
    }

    // start and clear OLED
    if(oled.begin(SSD1306_SWITCHCAPVCC, 0x3c, true, false)) {
        loggingSerial.println("SUCCESS - OLED started");
    }
    else {
        loggingSerial.println("FAILED - OLED start failed");
    }
    oled.clearDisplay();
    oled.display();

    // set pin modes of start button
    pinMode(Pin_Start_Button, INPUT);
    pinMode(Pin_Start_Button_LED, OUTPUT);
    digitalWrite(Pin_Start_Button_LED, LOW);

    // set pin modes for ultrasonic sensors
    for(uint8_t i = 0; i < 6; i++) {
        pinMode(Pins_UltraSonic_Echo[i], INPUT);
        pinMode(Pins_UltraSonic_Trig[i], OUTPUT);
        digitalWrite(Pins_UltraSonic_Trig[i], LOW);
    }

    // set pin modes for course mode switches
    pinMode(Pin_Obstacle_Switch, INPUT);
    pinMode(Pin_Test_Mode_Switch, INPUT);
    loggingSerial.println("SUCCESS - pin modes set");

    // wait for WRO Camera to get ready
    delay(3000);

    // start I2C as slave in fast mode
    if(i2c_slave.begin(0x51, Pin_I2C_SLAVE_SDA, Pin_I2C_SLAVE_SCL, 400000)) {
        loggingSerial.println("SUCCESS - I2C slave started");
    }
    else {
        loggingSerial.println("FAILED - I2C slave start failed");
    }
    i2c_slave.onReceive(i2cOnReceiveFunction);
    
    // start ultrasonic sensor thread
    if(xTaskCreatePinnedToCore(ultrasonicThreadFunction, "Ultrasonic Thread", 10000, NULL, 0, &ultrasonicThread, 1 - xPortGetCoreID()) == pdPASS) {
        loggingSerial.println("SUCCESS - Ultrasonic sensor thread created\n");
    }
    else {
        loggingSerial.println("FAILED - Ultrasonic sensor thread creation failed\n");
    }

    if(digitalRead(Pin_Test_Mode_Switch) == HIGH) {
        setLight(0, 1);
        setLight(1, 1);
        setLight(2, 1);
    }

    digitalWrite(Pin_Start_Button_LED, HIGH);
    loggingSerial.print(digitalRead(Pin_Test_Mode_Switch) ? (digitalRead(Pin_Obstacle_Switch) ? "Obstacle course" : "Starter course") : "Test mode");
    loggingSerial.println(" - Waiting for start signal ...");

    // wait for start signal
    while(digitalRead(Pin_Start_Button) == HIGH) {
        delay(1);
    }
    loggingSerial.println("Start signal received");

    if(digitalRead(Pin_Test_Mode_Switch) == HIGH) {
        //setServo(0, 7);
        antiRotation = rotation;
        startPosDistance = ultrasonicDistance[US_CenterFront];
    }
    else {
        loggingSerial.println("Test mode initialised\n");
        testAlgorithm();
    }
}

#pragma endregion setup


#pragma region loop

void loop() {
    if(digitalRead(Pin_Test_Mode_Switch) == HIGH) {
        //driveControl();
    }

    if(millis() > lastDisplayUpdate + 2000) {

        updateVoltageAndRPM();

        float batteryVoltage = (powerSensorData.analogValues[0] * 0.012060546875);
        float chargeLevel = (batteryVoltage - VOLTAGE_BATTERY_EMPTY) / (VOLTAGE_BATTERY_CHARGED - VOLTAGE_BATTERY_EMPTY);
        oled.clearDisplay();
        oled.setCursor(2, 2);
        oled.setTextSize(1);
        oled.setTextColor(1);
        oled.print(batteryVoltage, 2);
        oled.print(" V");
        oled.drawRect(80, 2, 40, 8, 1);
        oled.fillRect(80, 2, (uint8_t)(40 * (chargeLevel > 0 ? (chargeLevel < 1 ? chargeLevel : 1) : 0)), 8, 1);
        oled.setCursor(2, 16);
        if(cameraSensorData.object.available) {
            if(cameraSensorData.object.color) {
                oled.print("Red");
            }
            else {
                oled.print("Green");
            }
            if(cameraSensorData.object.direction) {
                oled.print(" - Right");
            }
            else {
                oled.print(" - Left");
            }
        }
        else {
            oled.print("No object");
        }
        oled.display();
        lastDisplayUpdate = millis();
    }

    delay(20);
}

#pragma endregion loop


#pragma region functions

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
                if((millis() > lastCurve + 2000) && (outsideBorder == Right) && (ultrasonicDistance[US_LeftFront] > 1100)) {
                    driveState.direction = Left;
                    driveState.state = Curve;
                    antiRotation = cameraSensorData.rotation - targetRotation;
                    targetRotation -= 900;
                    setServo(0, 11);
                    setServo(1, -15);
                    curveCount++;
                }
                if((millis() > lastCurve + 2000) && (outsideBorder == Left) && (ultrasonicDistance[US_RightFront] > 1100)) {
                    driveState.direction = Right;
                    driveState.state = Curve;
                    antiRotation = cameraSensorData.rotation - targetRotation;
                    targetRotation += 900;
                    setServo(0, 11);
                    setServo(1, 15);
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
        if((ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] > 40) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] < -40)) {
            driveState.direction = Left;
            driveState.state = UltrasonicCorrection;
            setServo(1, -3);
        }
        else if((driveState.state == UltrasonicCorrection) && (driveState.direction == Left) && (ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] < 20) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] > -20)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
        }
        else if((ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] < -40) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] > 40)) {
            driveState.direction = Right;
            driveState.state = UltrasonicCorrection;
            setServo(1, 3);
        }
        else if((driveState.state == UltrasonicCorrection) && (driveState.direction == Right) && (ultrasonicDistance[US_LeftFront] - ultrasonicDistance[US_LeftBack] > -20) && (ultrasonicDistance[US_RightFront] - ultrasonicDistance[US_RightBack] < 20)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
        }

        // near border correction (ultrasonic)
        if((driveState.state == BorderCorrection) && (ultrasonicDistance[US_LeftFront] > 150) && (ultrasonicDistance[US_RightFront] > 150)) {
            driveState.direction = Unknown;
            driveState.state = Unknown;
            setServo(1, 0);
        }
        if((ultrasonicDistance[US_LeftFront] < 120) && (ultrasonicDistance[US_LeftFront] != 0)) {
            driveState.direction = Right;
            driveState.state = BorderCorrection;
            setServo(1, 6);
        }
        else if((ultrasonicDistance[US_RightFront] < 120) && (ultrasonicDistance[US_RightFront] != 0)) {
            driveState.direction = Left;
            driveState.state = BorderCorrection;
            setServo(1, -6);
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
        for(uint8_t i = 0; i < 6; i++) {
            fireUltrasonic((UltraSonic_Process[i] == Variable) ? ((outsideBorder == Left) ? US_RightFront : ((outsideBorder == Right) ? US_LeftFront : US_CenterFront)) : UltraSonic_Process[i]);
            delay(30);
        }
    }
}

void setServo(uint8_t index, int8_t speed) {
    if(servoState[index] != speed) {
        servoState[index] = speed;
        i2c_master.beginTransmission(0x50);
        i2c_master.write(((index & 0x3) << 5) | ((speed < 0) * 0x10) | (((speed < 0) ? (-speed) : speed)) & 0xf);
        i2c_master.endTransmission();

        loggingSerial.print("Servo " + String(index, 10) + ": ");
        loggingSerial.println(speed, 10);
    }
}

void setLight(uint8_t index, bool state) {
    if((lightState & (1 << index)) >> index != state) {
        lightState ^= 1 << index;
        i2c_master.beginTransmission(0x50);
        i2c_master.write(0x80 | ((index & 0x3f) << 1) | state);
        i2c_master.endTransmission();

        loggingSerial.print("LED " + String(index, 10) + ": ");
        loggingSerial.println(state ? "ON" : "OFF");
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

void testAlgorithm() {

    // print ultrasonic sensor data
    for(uint8_t i = 0; i < 6; i++) {
        loggingSerial.println("Ultrasonic sensor " + String((uint32_t)i) + ": " + String(ultrasonicDistance[i] / 10.0, 1) + " cm");
    }

    // test LED functionality
    setLight(0, 1);
    delay(2000);
    setLight(0, 0);

    setLight(1, 1);
    delay(2000);
    setLight(1, 0);

    setLight(2, 1);
    delay(2000);
    setLight(2, 0);

    // test servo functionality
    for(uint8_t i = 0; i < 2; i++) {
        for(uint8_t j = 0; j < 4; j++) {
            for(int8_t k = 0; k < 16; k++) {
                setServo(i, j & 2 ? 0 - ( j & 1 ? 15 - k : k) : ( j & 1 ? 15 - k : k));
                delay(500);
            }
        }
    }
    
    // print power pcb sensor data
    updateVoltageAndRPM();
    loggingSerial.println("\nBattery Voltage: " + String(powerSensorData.analogValues[0] * 0.012060546875, 2) + " V");
    loggingSerial.println("Motor turns: " + String(powerSensorData.motorTurns[0] / 8.0, 3));

    // print camera data
    loggingSerial.println("Rotation: " + String(rotation / 10.0, 1) + "°");
}

#pragma endregion
