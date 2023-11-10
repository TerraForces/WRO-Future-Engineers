#ifndef SERVO_H
#define SERVO_H

/**
 * Servo library for ATMega328 MCUs
 * by Robert Flugrat
*/

#define SERVO_VERSION "1.0.0"

#include <Arduino.h>

// EN: formula for calculating the timer compare value from servo angle
// DE: Formel zur Berechnung des Timer-Vergleichswerts aus dem Servowinkel
#define SERVO_ANGLE_TO_256CLK(angle) (((::uint8_t)round((1500 + (angle * (1000.0 / 180))) / 16.0)) - 1)

// EN: data struct for maximum servo angle
// DE: Datenstruktur für maximale Servowinkel
struct SERVORANGE {

    //
    //
    ::int16_t min;

    //
    //
    ::int16_t max;
};

// EN: data struct for servo output infomation
// DE: Datenstruktur für Servo-Ausgangsinformationen
struct SERVO {

    // EN: ouput pin number
    // DE: Ausgangs-Pin-Nummer
    ::uint8_t pin;

    // EN: PWM pulse duration in 16 microseconds
    // DE: Dauer des PWM-Impulses in 16 Mikrosekunden
    ::uint8_t delay;

    //
    //
    ::SERVORANGE range;
};

// EN: data struct for information about all servos
// DE: Datenstruktur für Informationen zu allen Servos
struct SERVODATA {

    // EN: array of multiple servo information structs
    // DE: Array mehrerer Servoinformationsstrukturen
    ::SERVO* servos = NULL;

    // EN: number of active servo output pins
    // DE: Anzahl der aktiven Servo-Ausgangspins
    ::uint8_t initCount = 0;
};

// EN: global variable that contains all servo information
// DE: globale Variable, die alle Servoinformationen enthält
static ::SERVODATA servoData;

// EN: servo class
// DE: Servoklasse
class SERVOS {
    public:

        // EN: Function to set up 50Hz timer
        // DE: Funktion zum Einrichten des 50-Hz-Timers
        void init(const ::uint8_t count);

        // EN: function to add or change a servo output
        // DE: Funktion zum Hinzufügen oder Ändern eines Servoausgangs
        void set(::uint8_t pin, ::int16_t value, ::SERVORANGE range = {-90, 90});

        //
        //
        ::uint8_t maxCount = 0;
};

#endif
