#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "PIDController.hpp"


#define MILLISECONDS_TO_MICROSECONDS(X) (X * 1000)


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
                                            30.0, 2.0, 0);
mtrn3100::PIDController rotation_controller(mtrn3100::PIDMode::ROTATION, left_motor,
                                            left_motor_encoder, right_motor, right_motor_encoder,
                                            15.0, 5.0, 0); // changed these up a bit, was originally 30.0, 2.0, 0

void setup() {
    Serial.begin(9600);
}

// Set to 1 if u want to use the pid code , 0 if u want to use hardcoded
#define USE_HARDCODED_CODE 0

void loop() {
    #if USE_HARDCODED_CODE
    left_motor_encoder.setEncoderToZero();
    right_motor_encoder.setEncoderToZero();
    // turn 90 degrees to the right (cw)
    // the wheels need to turn 4.15 radians each, in order to travel 1/4th of the circle made by the axle
    left_motor.setPWM(-50);
    right_motor.setPWM(-50);
    while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.15) {
        continue;
    }
    left_motor.setPWM(0);
    right_motor.setPWM(0);

    delay(1000);

    left_motor_encoder.setEncoderToZero();
    right_motor_encoder.setEncoderToZero();
    // turn 90 degrees to the left
    left_motor.setPWM(50);
    right_motor.setPWM(50);
    while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.15) {
        continue;
    }
    left_motor.setPWM(0);
    right_motor.setPWM(0);

    delay(1000);

    left_motor_encoder.setEncoderToZero();
    right_motor_encoder.setEncoderToZero();
    // turn 360 degrees to the right
    left_motor.setPWM(-50);
    right_motor.setPWM(-50);
    while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.0 * 4.15) {
        continue;
    }
    left_motor.setPWM(0);
    right_motor.setPWM(0);

    delay(1000);

    left_motor_encoder.setEncoderToZero();
    right_motor_encoder.setEncoderToZero();
    // turn 360 degrees to the left
    left_motor.setPWM(50);
    right_motor.setPWM(50);
    while ((fabs(left_motor_encoder.getRotation()) + fabs(right_motor_encoder.getRotation())) / 2.0f < 4.0 * 4.15) {
        continue;
    }
    left_motor.setPWM(0);
    right_motor.setPWM(0);

    delay(1000);

    #else

    // turn 90 degrees to the right, pid loop stops after 5 seconds, hopefully thats enough time for it
    // to get into the right position
    rotation_controller.reset();
    rotation_controller.setTarget(-0.5 * PI);
    unsigned long prev_time = 0;
    const unsigned long sample_period = MILLISECONDS_TO_MICROSECONDS(10);
    while (prev_time < MILLISECONDS_TO_MICROSECONDS(5000)) {
        unsigned long curr_time = micros();

        if (curr_time - prev_time >= sample_period) {
            prev_time += sample_period;

            rotation_controller.update(0.01f);
        }
    }

    // turn 90 degrees to the left
    rotation_controller.reset();
    rotation_controller.setTarget(0.5 * PI);
    unsigned long prev_time = 0;
    const unsigned long sample_period = MILLISECONDS_TO_MICROSECONDS(10);
    while (prev_time < MILLISECONDS_TO_MICROSECONDS(5000)) {
        unsigned long curr_time = micros();

        if (curr_time - prev_time >= sample_period) {
            prev_time += sample_period;

            rotation_controller.update(0.01f);
        }
    }


    #endif

}
// position_controller.update(0.01f);


// delay(10);

// #if TEST_NUM == 1
    // should just move forward, then backward
    // left_motor.setPWM(-100);
    // right_motor.setPWM(100);
    // delay(2000);
    // left_motor.setPWM(100);
    // right_motor.setPWM(-100);

// #elif TEST_NUM == 2
// should move forward roughly 2 wheel turns = 20 cm
/*
    left_motor.setPWM(-70);
    right_motor.setPWM(70);
    while ((-left_motor_encoder.getRotation() + right_motor_encoder.getRotation()) / 2.0 < 2 * PI * 2) {
        Serial.print("L= ");
        Serial.println(left_motor_encoder.getRotation());
        Serial.print("R= ");
        Serial.println(right_motor_encoder.getRotation());

    }

    left_motor.setPWM(0);
    right_motor.setPWM(0);

    while (1) ;

}
*/




// PLEASE write a test for rotating left 90*//
//  test for rotating right 90*//
// test for full 360* rotation left, then full 360* rotation right //


// #elif TEST_NUM == 3

// // Should move 20 cm again, this time more accurately hopefully



// // while (true) {
// unsigned long now = micros();



// if (now - prev >= 1000) {
// prev = now;
// position_controller.update(DT);

// // Print encoder values for debugging
// // Serial.print("L=");
// // Serial.print(left_motor_encoder.getRotation());

// // Serial.print(" R=");
// // Serial.print(right_motor_encoder.getRotation());

// float pos = (left_motor_encoder.getRotation() +
// right_motor_encoder.getRotation()) / 2.0f;

// // Serial.print(" Avg=");
// // Serial.println(pos);

// // Stop when close enough to target
// if (abs(pos) > (2 * PI * 2)) {
// left_motor.setPWM(0);
// right_motor.setPWM(0);

// Serial.println("Target reached");

// while (1);
// }

// // Serial.println((left_motor_encoder.getRotation()+ right_motor_encoder.getRotation()) / 2.0f);

// }
// // }


// #endif

// =======

//     #elif TEST_NUM == 3

//     // Should move 20 cm again, this time more accurately hopefully
//     position_controller.setTarget(2 * PI * 2);
//     const float DT = 0.001f;
//     while (true) {
//         unsigned long prev = 0;
//         unsigned long now = micros();
//         if (now - prev >= 1000) {
//             prev = now;
//             position_controller.update(DT);
//         }
//     }


//     #endif
// >>>>>>> Stashed changes
// }