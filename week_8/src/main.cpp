#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"
#include "Miscellaneous.hpp"
#include "IMUGyroscope.hpp"
#include "Lidar.hpp"
#include "OLED.hpp"

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
const uint8_t LIDARRIGHT_EN_PIN = A1;
const uint8_t LIDARFRONT_EN_PIN = A2;
const uint8_t LIDARLEFT_EN_PIN = A0;
// const uint8_t

mm::Motor left_motor(LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN);
mm::Motor right_motor(RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN);
mm::Encoder left_motor_encoder(LEFT_MOTOR_ENC_A_PIN, LEFT_MOTOR_ENC_B_PIN, mm::EncoderSide::LEFT);
mm::Encoder right_motor_encoder(RIGHT_MOTOR_ENC_A_PIN, RIGHT_MOTOR_ENC_B_PIN, mm::EncoderSide::RIGHT);
mm::IMUGyroscope gyroscope{};
mm::PIDController position_controller(mm::PIDMode::POSITION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder, gyroscope, 
                                            55.0, 0.3, 0.15);
mm::PIDController rotation_controller(mm::PIDMode::ROTATION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder, gyroscope,
                                            60.0, 5, 0); // changed these up a bit, was originally 30.0, 2.0, 0
// for position, original values were 30.0, 2.0, 0
// for rotation original values were 15.0, 3.2, 0

mm::Lidar lidar1(LIDARFRONT_EN_PIN, 0x54);
//lidar1 is front

mm::Lidar lidar2(LIDARLEFT_EN_PIN, 0x55);
//lidar2 is left

mm::Lidar lidar3(LIDARRIGHT_EN_PIN, 0x56);
//lidar3 is right
mm::OLED oled{};


bool initOkay = false;

void setup() {
    Wire.begin();
    gyroscope.begin();
    oled.begin();
    Serial.begin(9600);


    Serial.println("Scanning...");
    initOkay = lidar1.init();
    if (!initOkay) {Serial.println("DID NOT CONNECT");};
    initOkay = lidar2.init();
    initOkay = lidar3.init();

    digitalWrite(LIDARLEFT_EN_PIN, LOW);
    digitalWrite(LIDARFRONT_EN_PIN, LOW);
}

mm::RotationController imu_controller{left_motor, right_motor, 10.0, 0.0, 0.0};

void loop() {
    mm::rotate(imu_controller, 90.0f);

    // if (initOkay) {
    //     int distance = lidar3.read();
    //     if (lidar3.timedOut()) {
    //         Serial.println("TIMEOUT");
    //     } else {
    //         Serial.println(distance);
    //     }
    // } else {
    //     Serial.println("NA");
    // }
    // delay(100);

}

    // delay(5000);

    // #if USE_HARDCODED_CODE
    // // Move forward 20 cm
    // left_motor_encoder.setEncoderToZero();
    // right_motor_encoder.setEncoderToZero();

    // left_motor.setPWM(-50);
    // right_motor.setPWM(50);
    // while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 2 * PI * 2) {
    //     continue;
    // }
    // left_motor.setPWM(0);
    // right_motor.setPWM(0);

    // delay(2000);

    // // Turn 90º CCW 4 times
    // for (uint8_t i {}; i < 4; i++) {
    //     left_motor_encoder.setEncoderToZero();
    //     right_motor_encoder.setEncoderToZero();

    //     left_motor.setPWM(50);
    //     right_motor.setPWM(50);
    //     while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.4) {
    //         continue;
    //     }
    //     left_motor.setPWM(0);
    //     right_motor.setPWM(0);

    //     delay(1000);
    // }

    // // Turn 90º CW 4 times
    // for (uint8_t i {}; i < 4; i++) {
    //     left_motor_encoder.setEncoderToZero();
    //     right_motor_encoder.setEncoderToZero();

    //     left_motor.setPWM(-50);
    //     right_motor.setPWM(-50);
    //     while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.4) {
    //         continue;
    //     }
    //     left_motor.setPWM(0);
    //     right_motor.setPWM(0);

    //     delay(1000);
    // }

    // #else

    // const unsigned long sample_period = MILLISECONDS_TO_MICROSECONDS(10);

    // // Move forward 20 cm
    // position_controller.reset();
    // position_controller.setTarget(14.3f);
    // unsigned long start_time = micros();
    // unsigned long prev_time = start_time;

    // while (micros() - start_time < 5000000UL) {
    //     if (micros() - prev_time >= sample_period) {
    //         prev_time += sample_period;
    //         position_controller.update(0.01f);
    //     }
    // }
    // left_motor.setPWM(0);
    // right_motor.setPWM(0);

    // // after forward motion
    // left_motor_encoder.setEncoderToZero();
    // right_motor_encoder.setEncoderToZero();

    // // Turn 90º CCW 4 times
    // for (uint8_t i {}; i < 4; i++) {
    //     rotation_controller.reset();
    //     rotation_controller.setTarget(0.25 * 2.0 * PI);
    //     start_time = micros();
    //     prev_time = start_time;

    //     while (micros() - start_time < 3800000UL) {
    //         if (micros() - prev_time >= sample_period) {
    //             prev_time += sample_period;
    //             rotation_controller.update(0.01f);
    //         }
    //     }

    //     left_motor.setPWM(0);
    //     right_motor.setPWM(0);
    // }

    // // Turn 90º CW 4 times
    // for (uint8_t i {}; i < 4; i++) {
    //     rotation_controller.reset();
    //     rotation_controller.setTarget(-0.25 * 2.0 * PI);
    //     start_time = micros();
    //     prev_time = start_time;

    //     while (micros() - start_time < 3800000UL) {
    //         if (micros() - prev_time >= sample_period) {
    //             prev_time += sample_period;
    //             rotation_controller.update(0.01f);
    //         }
    //     }

    //     left_motor.setPWM(0);
    //     right_motor.setPWM(0);
    // }

    // #endif

    // while (true) ;