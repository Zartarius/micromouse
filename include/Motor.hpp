#pragma once

#include <Arduino.h>

namespace mm {

static constexpr uint16_t COUNTS_PER_REVOLUTION = 700;

enum class MotorSide {
    LEFT, RIGHT
};

class Motor {
public:
    // pwm/dir: motor controller PWM and direction pins. enc1/enc2: encoder
    // channel A (interrupt pin, rising edge) and B pins. side selects which
    // encoder ISR this instance attaches.
    Motor(MotorSide side, uint8_t pwm, uint8_t dir, uint8_t enc1, uint8_t enc2);

    // abs(pwm) <= 255. Flipping the sign of pwm reverses direction.
    void setPWM(int16_t pwm);

    void setEncoderToZero(void);

    // Signed angular displacement of the motor (radians) since it was last zeroed.
    float getRotation(void);

private:
    void readEncoder(void);
    static void readEncoderLeftISR(void);
    static void readEncoderRightISR(void);

    const uint8_t pwm_pin, dir_pin;
    const uint8_t encoder1_pin, encoder2_pin;

    volatile long encoder_count = 0;

    static Motor* instance_left;
    static Motor* instance_right;
};

}
