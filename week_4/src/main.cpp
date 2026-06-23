#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"

/*
    We assume MOT1 in "Micromouse_Kit_Information.pdf" to refer to the left motor,
    and MOT2 to refer to the right motor.
*/
const uint8_t LEFT_MOTOR_PWM_PIN = 11;
const uint8_t LEFT_MOTOR_DIR_PIN = 12;
const uint8_t RIGHT_MOTOR_PWM_PIN = 9;
const uint8_t RIGHT_MOTOR_DIR_PIN = 10;
const uint8_t LEFT_MOTOR_ENC_A_PIN = 2;
const uint8_t LEFT_MOTOR_ENC_B_PIN = 7;
const uint8_t RIGHT_MOTOR_ENC_A_PIN = 3;
const uint8_t RIGHT_MOTOR_ENC_B_PIN = 8;

mtrn3100::Motor left_motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN);
mtrn3100::Motor right_motor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);
mtrn3100::Encoder left_motor_encoder(LEFT_MOTOR_ENC_A_PIN, LEFT_MOTOR_ENC_B_PIN, mtrn3100::EncoderSide::LEFT);
mtrn3100::Encoder right_motor_encoder(RIGHT_MOTOR_ENC_A_PIN, RIGHT_MOTOR_ENC_B_PIN, mtrn3100::EncoderSide::RIGHT);
mtrn3100::PIDController position_controller(mtrn3100::PIDMode::POSITION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder,
                                            200.0, 0, 0);
mtrn3100::PIDController rotation_controller(mtrn3100::PIDMode::ROTATION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder,
                                            200.0, 0, 0);

void setup() {

}

#define TEST_NUM 3

void loop() {
    #if TEST_NUM == 1
    // should just move forward, then backward
    left_motor.setPWM(-100);
    right_motor.setPWM(100);
    delay(2000);
    left_motor.setPWM(100);
    right_motor.setPWM(-100);

    #elif TEST_NUM == 2
    // should move forward roughly 2 wheel turns = 20 cm
    left_motor.setPWM(70);
    right_motor.setPWM(70);
    while ((left_motor_encoder.getRotation() + right_motor_encoder.getRotation()) / 2.0 < 2 * PI * 2) ;

    left_motor.setPWM(0);
    right_motor.setPWM(0);

    #elif TEST_NUM == 3

    // Should move 20 cm again, this time more accurately hopefully
    position_controller.setTarget(2 * PI * 5);
    const float DT = 0.001f;
    while (true) {
        unsigned long prev = 0;
        unsigned long now = micros();
        if (now - prev >= 1000) {
            prev = now;
            position_controller.update(DT);
        }
    }


    #endif
}
