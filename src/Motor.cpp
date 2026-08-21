#include "Motor.hpp"

namespace mm {

Motor* Motor::instance_left = nullptr;
Motor* Motor::instance_right = nullptr;

Motor::Motor(MotorSide side, uint8_t pwm, uint8_t dir, uint8_t enc1, uint8_t enc2)
    : pwm_pin{pwm}, dir_pin{dir}, encoder1_pin{enc1}, encoder2_pin{enc2} {
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    analogWrite(pwm_pin, 0);
    digitalWrite(dir_pin, LOW);

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

void Motor::setPWM(int16_t pwm) {
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

void Motor::setEncoderToZero(void) {
    // Interrupts must stay off while touching the volatile the ISR also writes.
    noInterrupts();
    encoder_count = 0;
    interrupts();
}

float Motor::getRotation(void) {
    noInterrupts();
    long count = encoder_count;
    interrupts();
    return (2.0f * PI * static_cast<float>(count)) / static_cast<float>(COUNTS_PER_REVOLUTION);
}

void Motor::readEncoder(void) {
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

void Motor::readEncoderLeftISR(void) {
    if (instance_left != nullptr) {
        instance_left->readEncoder();
    }
}

void Motor::readEncoderRightISR(void) {
    if (instance_right != nullptr) {
        instance_right->readEncoder();
    }
}

}
