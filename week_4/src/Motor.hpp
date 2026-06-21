#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {

class Motor {
public:
    /*
    Constructor for the Motor class
        - pwm_pin: A PWM pin on the Arduino board, which connects to the BIN2/BENABLE pin on the motor controller.
        The PWM output of this pin determines the motor speed.
        - dir_pin: A digital pin on the Arduino board, which connects to the BIN1/BPHASE pin on the motor controller.
        Switching between LOW and HIGH output switches the direction of rotation.
    */
    Motor(uint8_t pwm_pin, uint8_t in2) :  pwm_pin{pwm_pin}, dir_pin{in2} {
        pinMode(pwm_pin, OUTPUT);
        pinMode(in2, OUTPUT);

        // Set pins to a default value of 0
        analogWrite(pwm_pin, 0); // digitalWrite(pwm_pin, LOW) is also valid
        digitalWrite(in2, LOW);
    }


    /*
    Sets the motor speed for this motor instance
        - pwm: the pwm input for the motor, abs(pwm) <= 255. Flipping the sign of pwm will revers the direction
        of rotation of the motor.
    */
    void setPWM(int16_t pwm) {
        if (abs(pwm) > 255) {
            Serial.println("Invalid PWM value");
            return;
        }

        if (pwm >= 0) {
            digitalWrite(dir_pin, HIGH);
        } else {
            digitalWrite(dir_pin, LOW);
        }

        analogWrite(pwm_pin, abs(pwm));
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}
