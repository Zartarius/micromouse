#pragma once

#include "Robot.hpp"
#include "Miscellaneous.hpp"
#include "PIDController.hpp"

static mm::PIDController wall_centering_controller(0.8f, 0.0f, 0.0f);

namespace mm {

void robot_rotate(const float degrees, const unsigned long timeout_ms) {
    auto& robot = ROBOT;

    robot.rotation_controller.reset();
    robot.gyroscope.reset();

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float error = degrees - robot.gyroscope.getHeading();
    float prev_error = error;
    float error_derivative = 0.0f;

    while ((fabs(error) > 0.5f || fabs(error_derivative) > 15.0f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        robot.gyroscope.update();
        error = degrees - robot.gyroscope.getHeading();

        if (dt > 0.0f) {
            error_derivative = (error - prev_error) / dt;
        }
        prev_error = error;

        int16_t output = robot.rotation_controller.compute_output(error, dt, -80, 80);

        robot.left_motor.setPWM(-output);
        robot.right_motor.setPWM(output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


void robot_drive_straight(const float distance, const unsigned long timeout_ms) {
    auto& robot = ROBOT;

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 2.5f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);

        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -80, 80);

        robot.gyroscope.update();
        float heading_error = 0.0f - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output = constrain(forward_output - heading_output, -100, 100);
        int16_t right_output = constrain(forward_output + heading_output, -100, 100);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


// Note that the distance argument is still technically just referring to displacement
void robot_drive_straight_with_lidars(const float distance, const unsigned long timeout_ms) {
    auto& robot = ROBOT;

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(4.0f, 0.0f, 0.0f);

    const float turn_radians = distance / robot.wheel_radius_mm;
    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 2.5f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -80, 80);

        float left_dist = robot.lidar_system.readLeft();
        float right_dist = robot.lidar_system.readRight();
        bool left_wall = left_dist < (180 - 76);
        bool right_wall = right_dist < (180 - 76);

        // wall_centering_controller outputs a heading bias in degrees
        float heading_target = 0.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else if (left_wall) {
            float lateral_error = left_dist - 52;
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else if (right_wall) {
            float lateral_error = 52 - right_dist;
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else {
            wall_centering_controller.reset();
        }

        robot.gyroscope.update();
        float heading_error = heading_target - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output  = constrain(forward_output - heading_output, -100, 100);
        int16_t right_output = constrain(forward_output + heading_output, -100, 100);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}

}
