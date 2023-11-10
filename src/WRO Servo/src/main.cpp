/**
 * WRO Servo
 * by Robert Flugrat
*/

#define WRO_SERVO_VERSION "1.0.0"

// EN: enables test mode
// DE: Aktiviert den Testmodus
#define TEST_MODE

#pragma region includes

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <servo.h>

#pragma endregion includes


#pragma region pin_definitions

// EN: interrupt pins for counting motor turns
// DE: Interrupt-Anschlüsse zur Motordrehungszählung
#define Pins_Interrupt              (uint8_t[]){2, 3}

// EN: pins for controlling front LEDs
// DE: Anschlüsse der vorderen LEDs
#define Pins_LEDs                   (uint8_t[]){11, 10, 9, 8, 7, 6, 5, 4}

// EN: pins for sending PWM signal to motor controller and servos
// DE: Anschlüsse für Motor- und Servosignalausgabe
#define Pins_PWM                    (uint8_t[]){12, A1, A0, 13}

// EN: ADC pins for reading battery voltage
// DE: Analog-zu-Digital-Anschlüsse zum Auslesen der Batteriespannung
#define Pins_ADC                    (uint8_t[]){A2, A3, A6, A7}

#pragma endregion pin_definitions


#pragma region global_properties

// EN: I2C communication class
// DE: I2C-Kommunikationsklasse
TwoWire i2c = Wire;

// EN: servo controlling class
// DE: Klasse zur Servosteuerung
SERVOS servo;

// EN: count of motor turns since last reset
// DE: Anzahl der Motorumdrehungen seit dem letzten Zurücksetzen
uint32_t motorTurns1 = 0;
uint32_t motorTurns2 = 0;

// EN: start time of motor turn counting (milliseconds)
// DE: Zeit vom Start der Motorumdrehungszählung (Millisekunden)
uint64_t motorTurnsStartMillis = 0;

// EN: I2C request response function active flag
// DE: Aktivitätsmarkierung der I2C-Anfragenbeantwortungsfunktion
bool i2cRequestWorking = false;

#pragma endregion global_properties


#pragma region functions

// EN: motor turns counting interrupt function
// DE: Interruptfunktion zur Motorumdrehungszählung
void interruptFunction1();
void interruptFunction2();

// EN: I2C request response function guard function to prevent multiple I2C request response functions working
// DE: I2C-Anfragenbeantwortungfunktionsschutzfunktion zur Verhinderung der Ausführung von mehreren I2C-Anfragenbeantwortungsfunktionen
void i2cOnRequestGuardFunction();

// EN: I2C request response function
// DE: I2C-Anfragenbeantwortungsfunktion
void i2cOnRequestFunction();

// EN: I2C command receive function
// DE: I2C-Befehlempfangsfunktion
void i2cOnReceiveFunction(int bytes);

#pragma endregion functions


// EN: main function, executed once on program start
// DE: Hauptfunktion, wird einmalig beim Programmstart ausgeführt
void setup() {

    // EN: start serial (UART) communication with PC for logging at 115200 bit per second
    // DE: Starten der seriellen (UART) Kommunikation mit einem PC zur Protokollierung mit 115200 Bit pro Sekunde
    Serial.begin(115200);

    // EN: print program name and version to serial port
    // DE: Ausgabe des Programmnamens und der Programmversion auf dem seriellen Anschluss
    Serial.println("WRO Servo");
    Serial.println(String("Version ") + WRO_SERVO_VERSION);
    Serial.println();

    // EN: set I2C clock to 400 kHz
    // DE: Setzen der I2C-Uhr zu 400 kHz
    i2c.setClock(400000);

    // EN: start I2C communication in slave mode at address 0x50 (hexadecimal)
    // DE: Starten der I2C-Kommunikation im Slave-Modus auf Adresse 0x50 (hexadezimal)
    i2c.begin(0x50);

    // EN: set pin modes of motor turn counting interrupt pins
    // DE: Setzen der Anschluss-Modi der Motorumdrehungszählungs-Interrupt-Anschlüsse
    for(uint8_t i = 0; i < 2; i++) {
        pinMode(Pins_Interrupt[i], INPUT);
    }

    // EN: set pin modes of front LED controlling pins
    // DE: Setzen der Anschluss-Modi der Steuerungsanschlüsse der vorderen LEDs
    for(uint8_t i = 0; i < 8; i++) {
        pinMode(Pins_LEDs[i], OUTPUT);
        digitalWrite(Pins_LEDs[i], LOW);
    }

    // EN: start servo class with memory for 4 servos
    // DE: Starten der Servoklasse mit Speicher für 4 Servomotoren
    //servo.init(4);

    // EN: output 0° signal to all servo pins
    // DE: Ausgabe des 0°-Signales an allen Servomotoranschlüssen
    for(uint8_t i = 0; i < 4; i++) {
        //servo.set(Pins_PWM[i], 0);
    }

    // EN: set pin modes of ADCs
    // DE: Setzen der Anschluss-Modi der Analog-zu-Digital-Anschlüsse
    for(uint8_t i = 0; i < 4; i++) {
        pinMode(Pins_ADC[i], INPUT);
    }

    // EN: attach interrupts for counting motor turns
    // DE: Setzen der Interrupts zur Motorumdrehungszählung
    attachInterrupt(digitalPinToInterrupt(Pins_Interrupt[0]), interruptFunction1, RISING);
    attachInterrupt(digitalPinToInterrupt(Pins_Interrupt[1]), interruptFunction2, RISING);
    motorTurnsStartMillis = millis();

    // EN: set i2c interrupt functions
    // DE: Setzen der I2C-Interrupt-Funktionen
    i2c.onRequest(i2cOnRequestGuardFunction);
    i2c.onReceive(i2cOnReceiveFunction);
    Serial.println("3");
}

// EN: main function, executed permanently after setup()
// DE: Hauptfunktion, wird dauerhaft nach setup() ausgeführt
void loop() {}

// EN: motor turns counting interrupt function
// DE: Interruptfunktion zur Motorumdrehungszählung
void interruptFunction1() {

    // EN: incrase the count of motor turns since last reset
    // DE: Erhöhen der Anzahl der Motorumdrehungen seit dem letzten Zurücksetzen
    motorTurns1++;
}

// EN: motor turns counting interrupt function
// DE: Interruptfunktion zur Motorumdrehungszählung
void interruptFunction2() {

    // EN: incrase the count of motor turns since last reset
    // DE: Erhöhen der Anzahl der Motorumdrehungen seit dem letzten Zurücksetzen
    motorTurns2++;
}

// EN: I2C request response function guard function to prevent multiple I2C request response functions working
// DE: I2C-Anfragenbeantwortungfunktionsschutzfunktion zur Verhinderung der Ausführung von mehreren I2C-Anfragenbeantwortungsfunktionen
void i2cOnRequestGuardFunction() {

    Serial.println("I2C request guard");

    // EN:
    // DE:
    if(!i2cRequestWorking){

        // EN:
        // DE:
        i2cRequestWorking = true;

        // EN:
        // DE:
        i2cOnRequestFunction();

        // EN:
        // DE:
        i2cRequestWorking = false;
    }
}

// EN: I2C request response function
// DE: I2C-Anfragenbeantwortungsfunktion
void i2cOnRequestFunction() {

    Serial.println("I2C request");

    // EN:
    // DE:
    uint16_t buffer[6];

    // EN:
    // DE:
    for(uint8_t i = 0; i < 4; i++){

        // EN:
        // DE: 
        buffer[i] = analogRead(Pins_ADC[i]);
    }
    
    // EN:
    // DE:
    buffer[4] = (uint16_t)((motorTurns1 * 100000.0) / motorTurnsStartMillis); // 0,01 turns per second
    buffer[5] = (uint16_t)((motorTurns2 * 100000.0) / motorTurnsStartMillis); // 0,01 turns per second

    // EN:
    // DE:
    i2c.write((char*)buffer, 12);
}

// EN: I2C command receive function
// DE: I2C-Befehlempfangsfunktion
void i2cOnReceiveFunction(int bytes) {

    Serial.println("I2C receive");

    // EN:
    // DE:
    for(uint8_t i = 0; i < bytes; i++) {

        // EN:
        // DE:
        uint8_t command = i2c.read();

        // EN:
        // DE:
        if(command & 0b10000000) {

            // EN:
            // DE:
            digitalWrite(Pins_LEDs[(command & 0b1111110) >> 1], command & 0b1);
        }

        // EN:
        // DE:
        else {

            // EN:
            // DE:
            servo.set(Pins_PWM[(command & 0b1100000) >> 5], command & 0b10000 ? ((command & 0b1111) * 6) : -((command & 0b1111) * 6));
        }
    }
}
