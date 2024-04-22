#include "servo.h"

uint8_t nextServo = 0;

ISR(TIMER1_COMPA_vect) {
    if(servoData.initCount > 1) { 
        cli();
        OCR2A = servoData.servos[0].delay;
        TIMSK2 |= 1 << OCIE2A;
        TCCR2A = 1 << WGM21;
        TCCR2B = (1 << CS22) | (1 << CS21);
        sei();
        nextServo = 0;
    }
}

ISR(TIMER2_COMPA_vect) {
    ::digitalWrite(servoData.servos[nextServo].pin, LOW);
    nextServo++;
    if(nextServo >= servoData.initCount) {
        cli();
        TIMSK2 ^= 1 << OCIE2A;
        TCCR2A = 0;
        TCCR2B = 0;
        TCNT2 = 0;
        sei();
    }
    else {
        ::digitalWrite(servoData.servos[nextServo].pin, HIGH);
        cli();
        OCR2A = servoData.servos[nextServo].delay;
        sei();
    }
}

void SERVOS::init(const uint8_t count) {
    servoData.servos = new SERVO[count + 1];
    maxCount = count + 1;

    cli();
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    TCNT1 = 0;
    OCR1A = 39999;
    TIMSK1 |= 1 << OCIE1A;
    sei();

    this->set(255, 500);
}

void SERVOS::set(uint8_t pin, int16_t value, SERVORANGE range) {
    uint8_t index = 255;
    for(uint8_t i = 0; i < servoData.initCount; i++) {
        if(servoData.servos[i].pin == pin) {
            index = i;
        }
    }
    if(index != 255) {
        servoData.servos[index].delay = SERVO_ANGLE_TO_256CLK(value);
        servoData.servos[index].range = range;
    }
    else if(servoData.initCount < maxCount) {
        servoData.servos[servoData.initCount].pin = pin;
        servoData.servos[servoData.initCount].delay = SERVO_ANGLE_TO_256CLK(value);
        servoData.servos[servoData.initCount].range = range;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        servoData.initCount++;
    }
}
