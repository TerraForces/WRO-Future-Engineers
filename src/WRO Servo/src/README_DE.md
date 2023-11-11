# WRO Future Enginiers - Team TerraForce - Arduino Nano

## Headerdateien einbinden
```
#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
```
Diese Dateien des Arduino Frameworks stellen allgemeine Funktionen, sowie Funktionen und Klassen für die serielle und I2C-Kommunikation bereit.

```
#include <servo.h>
```
Diese eigene [Headerdatei](</src/WRO Servo/lib/Servo/README_DE.md>) enthält eine Klasse zur Servoansteuerung.

## Anschlussdefinitionen
```
#define Pins_Interrupt              (uint8_t[]){2, 3}
#define Pins_LEDs                   (uint8_t[]){11, 10, 9, 8, 7, 6, 5, 4}
#define Pins_PWM                    (uint8_t[]){12, A1, A0, 13}
#define Pins_ADC                    (uint8_t[]){A2, A3, A6, A7}
```
Hier werden die verschiedenen benutzten Anschlüsse für
* die Interrupts zur Motorumdrehungszählung
* die LEDs am Darlington-Array
* die Servosignal-Ausgabe für die Motoren
* die Analog-Digital-Wandler zur Batteriespannungsmessung

aufgezählt.

## Globale Variablen
```
TwoWire i2c = Wire;
```
Hier wird ein Objekt der TwoWire-Klasse für die I2C-Kommunikation erzeugt.
```
SERVOS servo;
```
Dies ist ein Objekt der Klasse SERVOS aus der Headerdatei [Servo.h](</src/WRO Servo/lib/Servo/README_DE.md>).
```
uint32_t motorTurns1 = 0;
uint32_t motorTurns2 = 0;
```
In diesen Variablen wird die Anzahl der Motorumdrehungen seit dem letzten Reset gespeichert.
```
uint64_t motorTurnsStartMillis = 0;
```
In dieser Variable wird die Zeit in Millisekunden seit dem letzten Reset der Motorumdrehungszählung abgespeichert.
```
bool i2cRequestWorking = false;
```
Dieses Boolean gibt an, ob gerade die Beantwortungsfunktion für I2C-Anfragen aktiv ist.