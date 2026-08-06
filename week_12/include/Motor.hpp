#pragma once

#include <Arduino.h>

namespace mm {

static constexpr uint16_t COUNTS_PER_REVOLUTION = 700;

enum class MotorSide {
    LEFT, RIGHT
};

class Motor {
public:
    /*
    Constructor for the Motor class
        - pwm_pin: A PWM pin on the Arduino board, which connects to the BIN2/BENABLE pin on the motor controller.
        The PWM output of this pin determines the motor speed.
        - dir_pin: A digital pin on the Arduino board, which connects to the BIN1/BPHASE pin on the motor controller.
        Switching between LOW and HIGH output switches the direction of rotation.
        - enc1: An interrupt pin on the Arduino board, connected to channel A of the motor encoder
        by convention. The ISR is called at every rising edge on this pin.
        - enc2: A digital (input) pin on the arduino board, connected to the other channel of the
        motor encoder (usually channel B). Used to derive the direction of rotation.
        - side: The encoder which this instance is for. Two Encoder instances should be created (one left and one right)
        if you want the left and right wheels to be controlled separately.
    */
    Motor(MotorSide side, uint8_t pwm, uint8_t dir, uint8_t enc1, uint8_t enc2)
        : pwm_pin{pwm}, dir_pin{dir}, encoder1_pin{enc1}, encoder2_pin{enc2} {
        // Motor setup
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
        analogWrite(pwm_pin, 0); // digitalWrite(pwm_pin, LOW) is also valid
        digitalWrite(dir_pin, LOW);

        // Encoder setup
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (side == MotorSide::LEFT) {
            instance_left = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderLeftISR, RISING);
        } else if (side == MotorSide::RIGHT) {
            instance_right = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderRightISR, RISING);
        }
    }


    /*
    Sets the motor speed for this motor instance
        - pwm: the pwm input for the motor, abs(pwm) <= 255. Flipping the sign of pwm will revers the direction
        of rotation of the motor.
    */
    void setPWM(int16_t pwm) {
        if (this == instance_left) {
            if (pwm >= 0) {
                digitalWrite(dir_pin, LOW);
            } else {
                digitalWrite(dir_pin, HIGH);
            }
        } else if (this == instance_right) {
            if (pwm >= 0) {
                digitalWrite(dir_pin, HIGH);
            } else {
                digitalWrite(dir_pin, LOW);
            }
        }

        analogWrite(pwm_pin, (pwm >= 0 ? pwm : -pwm));
    }

    /*
    Sets the encoder_count of the Encoder instance to 0. Useful when you want to reset
    the encoder for different stages of movement.
    */
    void setEncoderToZero() {
        // We need to turn off interrupts when modifying volatile variables
        // to prevent data races
        noInterrupts();
        encoder_count = 0;
        interrupts();
    }

    /*
    Returns the signed angular displacement of the motor (in radians) since it was last set to 0.
    */
    float getRotation() {
        noInterrupts();
        long count = encoder_count;
        interrupts();
        return (2.0f * PI * static_cast<float>(count)) / static_cast<float>(COUNTS_PER_REVOLUTION);
    }

private:
    /*
    Called by the ISR to increment/decrement the encoder_count
    */
    void readEncoder() {
        if (this == instance_left) {
            if (digitalRead(encoder2_pin) == HIGH) {
                encoder_count--;
            } else {
                encoder_count++;
            }
        } else if (this == instance_right) {
            if (digitalRead(encoder2_pin) == HIGH) {
                encoder_count++;
            } else {
                encoder_count--;
            }
        }
    }

    /*
    The ISR for the left encoder.
    */
    static void readEncoderLeftISR() {
        // Ensure the instance has actually been created
        if (instance_left != nullptr) {
            instance_left->readEncoder();
        }
    }

    /*
    The ISR for the right encoder.
    */
    static void readEncoderRightISR() {
        // Ensure the instance has actually been created
        if (instance_right != nullptr) {
            instance_right->readEncoder();
        }
    }

    const uint8_t pwm_pin, dir_pin;
    const uint8_t encoder1_pin, encoder2_pin;

    volatile long encoder_count = 0;

    static Motor* instance_left;
    static Motor* instance_right;
};

Motor* Motor::instance_left = nullptr;
Motor* Motor::instance_right = nullptr;

}
