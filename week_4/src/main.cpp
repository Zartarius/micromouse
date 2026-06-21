#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"

/*
    We assume MOT1 in "Micromouse_Kit_Information.pdf" to refer to the left motor,
    and MOT2 to refer to the right motor.
*/
const int LEFT_MOTOR_PWM_PIN = 14;
const int LEFT_MOTOR_DIR_PIN = 15;
const int RIGHT_MOTOR_PWM_PIN = 12;
const int RIGHT_MOTOR_DIR_PIN = 13;
const int LEFT_MOTOR_ENC_A_PIN = 5;
const int LEFT_MOTOR_ENC_B_PIN = 10;
const int RIGHT_MOTOR_ENC_A_PIN = 6;
const int RIGHT_MOTOR_ENC_B_PIN = 11;

mtrn3100::Motor left_motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN);
mtrn3100::Motor right_motor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);
mtrn3100::Encoder left_motor_encoder(LEFT_MOTOR_ENC_A_PIN, LEFT_MOTOR_ENC_B_PIN, mtrn3100::EncoderSide::LEFT);
mtrn3100::Encoder right_motor_encoder(RIGHT_MOTOR_ENC_A_PIN, RIGHT_MOTOR_ENC_B_PIN, mtrn3100::EncoderSide::RIGHT);

void setup() {

}

void loop() {


}
