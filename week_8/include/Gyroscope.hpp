#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include "Motor.hpp"
#include "Miscellaneous.hpp"

/*
    Set this to a non-zero value if you want to use a moving average
    filter to calculate IMU readings.
*/
#define USE_MOVING_AVERAGE_FILTER 0

namespace mm {

#if USE_MOVING_AVERAGE_FILTER
/*
    Size of the window used in moving average filter calculations. Keep within
    the range 5-25.
*/
static constexpr uint8_t WINDOW_SIZE = 5;
#endif

class Gyroscope {
public:
#if USE_MOVING_AVERAGE_FILTER
    Gyroscope(void) {
        for (int i = 0; i < WINDOW_SIZE; i++) {
            ring_buffer.buffer[i] = 0.0f;
        }
    }
#else
    Gyroscope(void) = default;
#endif
    /*
    Initialises the MPU, blocks indefinitely if unable to connect to
    the MPU. Call this method at the beginning of main.cpp:setup.
    */
    void begin(void) {
        if (mpu6050.begin() != 0) {
            Serial.println("Could not connect to MPU6050");
            while (true);
        }
        Wire.beginTransmission(0x68);
        Wire.write(0x1B); // gyro config register
        Wire.write(0x08); // Sets to range 1 (±500 deg/s)
        Wire.endTransmission();

        mpu6050.calcOffsets();
        // todo: possibly hardcode zero bias
    }

    /*
    Resets the IMU's gyroscope calibration, use whenever changing targets.
    */
    void reset(void) {
        mpu6050.resetAngles();
    }

    /*
    Updates the IMU, call as often as possible, ideally >= 20 Hz.
    */
    void update(void) {
        mpu6050.update();

        #if USE_MOVING_AVERAGE_FILTER
            ring_buffer.push(getHeading());
        #endif
    }

    /*
    Returns heading of the IMU, yet to determine units etc.
    */
    float getHeading() {
        #if USE_MOVING_AVERAGE_FILTER
            float average_heading = 0.0f;

            size_t idx = ring_buffer.buffer_head;
            while (idx != ring_buffer.buffer_tail) {
                average_heading += ring_buffer.buffer[idx];
                idx = (idx + 1) % WINDOW_SIZE;
            }

            return average_heading / static_cast<float>(WINDOW_SIZE);
        #else
            return mpu6050.getAngleZ() - bias;
        #endif
    }

private:
    MPU6050 mpu6050{Wire};
    float bias{0.0f};
    #if USE_MOVING_AVERAGE_FILTER
    mm::RingBuffer<float, WINDOW_SIZE> ring_buffer;
    #endif
};


// class RotationController {
// public:
//     RotationController(IMUGyroscope& gyroscope, Motor& left_motor, Motor& right_motor,
//                         float kp, float ki, float kd)
//                     : gyroscope{gyroscope}, left_motor{left_motor}, right_motor{right_motor}, kp{kp}, ki{ki}, kd{kd} {}

//     void setTarget(float target) {
//         this->target = target;

//         state.integral = 0.0f;
//         state.prev_error = 0.0f;

//         gyroscope.reset();
//     }

//     float getError(void) {
//         return target - gyroscope.getHeading();
//     }

//     void update(float dt) {
//         float error = getError();
//         float output = pid_compute(dt, error);
//         output = constrain(output, -255, 255);

//         left_motor.setPWM(static_cast<int16_t>(output));
//         right_motor.setPWM(static_cast<int16_t>(output));
//     }

//     void stop(void) {
//         left_motor.setPWM(0);
//         right_motor.setPWM(0);
//     }

// private:
//     /*
//         State of PID control, includes current integral value, previous error and the max integral absolute value
//     */
//     struct PIDState {
//         float integral = 0.0f;
//         float prev_error = 0.0f;
//         float integral_limit = 100.0f;
//     };

//     /*
//         Computes a pid output given the state and parameters
//     */
//     float pid_compute(float dt, float error) {
//         state.integral += error * dt;
//         state.integral = constrain(state.integral, -state.integral_limit, state.integral_limit);
//         float derivative = (error - state.prev_error) / dt;
//         state.prev_error = error;
//         return (kp * error) + (ki * state.integral) + (kd * derivative);
//     }

//     IMUGyroscope& gyroscope;

//     Motor& left_motor;
//     Motor& right_motor;

//     float kp, ki, kd;
//     float target = 0.0f;

//     PIDState state;


//     // to remove later
//     friend void rotate(RotationController& controller, float target);
// };

// void rotate(RotationController& controller, float target) {
//     controller.setTarget(target);

//     unsigned long start = micros();
//     unsigned long prev = start;

//     while (micros() - start < MILLISECONDS_TO_MICROSECONDS(3000)) {
//         controller.gyroscope.update();

//         if (micros() - prev >= MILLISECONDS_TO_MICROSECONDS(10)) {
//             prev += MILLISECONDS_TO_MICROSECONDS(10);
//             controller.update(0.01f);
//         }

//         // Serial.println(controller.getError());
//         // Serial.println(controller.gyroscope.getHeading());
//         Serial.println(controller.getError());
//     }

//     controller.stop();
// }

}

