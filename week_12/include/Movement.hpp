#pragma once

#include "Robot.hpp"
#include "Miscellaneous.hpp"
#include "PIDController.hpp"

namespace mm {

void robot_stop(void) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}

void robot_rotate(const float degrees, const unsigned long timeout_ms) {
    auto& robot = GET_ROBOT();

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
    auto& robot = GET_ROBOT();

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
void robot_drive_straight_with_lidars(const float distance, const unsigned long timeout_ms, bool stop_at_end = true) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(1.2f, 0.0f, 0.05f);

    const float turn_radians = distance / robot.wheel_radius_mm;
    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 2.5f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        pos_error = turn_radians - ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -100, 100);

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


        float heading_error = heading_target - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output  = constrain(forward_output - heading_output, -140, 140);
        int16_t right_output = constrain(forward_output + heading_output, -140, 140);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    if (stop_at_end) {
        robot.left_motor.setPWM(0);
        robot.right_motor.setPWM(0);
    }
}

void robot_turn(const float degrees, const float radius_mm, const unsigned long timeout_ms) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.gyroscope.reset();

    const float half_track_width = robot.track_width_mm / 2.0f;

    const float left_to_right_ratio = (degrees >= 0.0f) ?
            (radius_mm - half_track_width) / (radius_mm + half_track_width) :
            (radius_mm + half_track_width) / (radius_mm - half_track_width);

    const int16_t left_speed = 60;
    const int16_t right_speed = static_cast<int16_t>(static_cast<float>(left_speed) / left_to_right_ratio);

    unsigned long start_time = micros();
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    unsigned long last_update_ms = millis();
    bool turn_complete = false;

    while (!turn_complete && micros() - start_time < timeout) {
        unsigned long now_ms = millis();

        // Only poll the gyro once a real millisecond has actually elapsed,
        // so MPU6050_light's internal dt is never zero.
        if (now_ms != last_update_ms) {
            last_update_ms = now_ms;
            robot.gyroscope.update();
            turn_complete = (fabs(robot.gyroscope.getHeading()) >= fabs(degrees));
        }

        robot.left_motor.setPWM(left_speed);
        robot.right_motor.setPWM(right_speed);
    }
}


void optimised_chaining(const char *movement) {
    auto& robot = GET_ROBOT();

    constexpr float CELL = 180.0f;
    constexpr float HALF_CELL = CELL / 2.0f;
    constexpr float TURN_RADIUS = HALF_CELL;

    const auto& count_forwards = [](const char *p) {
        int n = 0;
        while (*p == 'f') {
            n++;
            p++;
        }
        return n;
    };

    int i = 0;

    while (movement[i] != '\0') {
        // Ideally should never get here
        if (movement[i] == 'l' || movement[i] == 'r') {
            robot_rotate(movement[i] == 'l' ? 90.0f : -90.0f, 3000);
            i++;
            continue;
        }

        int num_f = count_forwards(movement + i);

        if (movement[i + num_f] != 'l' && movement[i + num_f] != 'r') {
            // String ends
            robot_drive_straight_with_lidars((float) num_f * CELL, 15000);
            break;
        }

        robot_drive_straight_with_lidars(((float) num_f - 1.0f) * CELL + HALF_CELL, 15000);
        i += (num_f - 1);

        // 90º turn left
        if (str_eq(&movement[i], "flf", 3)) {
            robot_turn(90.0f, TURN_RADIUS, 5000);
            i += 3;
        } else if (str_eq(&movement[i], "frf", 3)) {
            robot_turn(-90.0f, TURN_RADIUS, 5000);
            i += 3;
        } else if (str_eq(&movement[i], "flflf", 5)) {
            robot_turn(180.0f, TURN_RADIUS, 10000);
            i += 5;
        } else if (str_eq(&movement[i], "frfrf", 5)) {
            robot_turn(-180.0f, TURN_RADIUS, 10000);
            i += 5;
        } else {
            robot.oled.print(0, 0, "wtf");
            while (true) ;
        }

        robot.left_motor.setEncoderToZero();
        robot.right_motor.setEncoderToZero();
        while ((robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f < (HALF_CELL / robot.wheel_radius_mm)) {
            robot.left_motor.setPWM(80);
            robot.right_motor.setPWM(80);
        }
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}

}
