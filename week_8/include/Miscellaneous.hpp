#pragma once

#include "Robot.hpp"

#define MILLISECONDS_TO_MICROSECONDS(X) ((unsigned long)(X) * 1000UL)

namespace mm {

/*
    Same functionality as delay(), but it regularly updates the gyroscope too.
    Use this instead of delay() everywhere once the gyroscope is initialised.
*/
void delayWhileUpdating(unsigned long duration_ms) {
    auto& robot = ROBOT;

    unsigned long start = millis();
    while (millis() - start < duration_ms) {
        robot.gyroscope.update();
    }
}

/*
    Ringbuffer data structure, for use in moving average filters and other stuff.
*/
template <typename T, size_t len>
struct RingBuffer {
public:
    bool push(const T& item) {
        if (isFull()) return false;
        buffer[buffer_head] = item;
        buffer_head = (buffer_head + 1) % len;
        count++;
        return true;
    }

    bool pop(T& item) {
        if (isEmpty()) return false;
        item = buffer[buffer_tail];
        buffer_tail = (buffer_tail + 1) % len;
        count--;
        return true;
    }

    bool peek(T& item) const {
        if (isEmpty()) return false;
        item = buffer[buffer_tail];
        return true;
    }

    bool isEmpty() const { return count == 0; }
    bool isFull() const { return count == len; }
    size_t size() const { return count; }

    void clear() {
        buffer_head = 0;
        buffer_tail = 0;
        count = 0;
    }

    T buffer[len];
    size_t buffer_head = 0;
    size_t buffer_tail = 0;
    size_t count = 0;
};



}





/*
    FUNCTIONS/CLASSES TO MOVE LATER
*/

#include "PIDController.hpp"
#include "Gyroscope.hpp"
#include "LidarSystem.hpp"

void rotate(mm::PIDController& rotation_controller, float degrees, bool reset_gyroscope = true) {
    auto& robot = ROBOT;

    rotation_controller.reset();

    if (reset_gyroscope) {
        robot.gyroscope.reset();
    }

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(3000);

    float error = degrees - robot.gyroscope.getHeading();
    float prev_error = error;
    float error_derivative = 0.0f;

    while ((fabs(error) > 2.5f || fabs(error_derivative) > 15.0f)
            && micros() - start_time < timeout) {

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        robot.gyroscope.update();
        error = degrees - robot.gyroscope.getHeading();

        if (dt > 0.0f) {
            error_derivative = (error - prev_error) / dt;
        }
        prev_error = error;

        int16_t output = rotation_controller.compute_output(error, dt, -80, 80);
        robot.left_motor.setPWM(output);
        robot.right_motor.setPWM(output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


// distance must be in mm
void driveStraight(mm::PIDController& position_controller,
                mm::PIDController& heading_controller,
                float distance) {
    auto& robot = ROBOT;

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();

    float wheel_turns = distance / 16.0f;
    position_controller.reset();
    heading_controller.reset();
    robot.gyroscope.reset();

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(3000);

    float pos_error = wheel_turns - ((-robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
    // float prev_pos_error = pos_error;
    // // float pos_derivative = 0.0f;

    while (/*(fabs(pos_error) > 0.7f || fabs(pos_derivative) > 2.5f) &&*/
            micros() - start_time < timeout) {

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        pos_error = wheel_turns - ((-robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);

        // if (dt > 0.0f) {
        //     // pos_derivative = (pos_error - prev_pos_error) / dt;
        // }
        // // prev_pos_error = pos_error;

        int16_t forward_output = position_controller.compute_output(pos_error, dt, -80, 80);

        robot.gyroscope.update();
        float heading_error = 0.0f - robot.gyroscope.getHeading();
        int16_t heading_output = heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output = -forward_output + heading_output; // maybe do - heading_output if this doesn't work
        int16_t right_output = forward_output + heading_output;
        left_output = constrain(left_output, -80, 80);
        right_output = constrain(right_output, -80, 80);

        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}
