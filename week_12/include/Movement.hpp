#pragma once

#include "Robot.hpp"
#include "Misc.hpp"
#include "PIDController.hpp"

namespace mm {

void robot_stop(void) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}

void robot_rotate(const float degrees, const unsigned long timeout_ms, const int16_t max_abs_pwm) {
    auto& robot = GET_ROBOT();

    robot.rotation_controller.reset();
    robot.gyroscope.reset();

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float error = degrees - robot.gyroscope.getHeading();
    float prev_error = error;
    float error_derivative = 0.0f;

    // lower = smoother to react, higher = noisier to react
    const float derivative_smoothing = 0.2f;

    while ((fabs(error) > 0.5f || fabs(error_derivative) > 20.0f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        robot.gyroscope.update();
        error = degrees - robot.gyroscope.getHeading();

        if (dt > 0.0f) {
            float raw_derivative = (error - prev_error) / dt;
            // Single noisy sample can't spike error_derivative and
            // stall the exit condition.
            error_derivative = error_derivative * (1.0f - derivative_smoothing)
                              + raw_derivative * derivative_smoothing;
        }
        prev_error = error;

        int16_t output = robot.rotation_controller.compute_output(error, dt, -max_abs_pwm, max_abs_pwm);

        robot.left_motor.setPWM(-output);
        robot.right_motor.setPWM(output);
    }

    robot_stop();
}


void robot_drive_straight(
    const float distance,
    const unsigned long timeout_ms,
    const float max_speed_mm_s = 300.0f,
    const float accel_mm_s2 = 900.0f
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();

    const float turn_radians = distance / robot.wheel_radius_mm;

    // Trapezoidal velocity profile over wheel-rotation radians: ramp up to
    // peak_velocity at `accel`, cruise, then ramp back down. Falls back to
    // a triangle profile (never reaching peak_velocity) on short moves.
    // Tune max_speed_mm_s / accel_mm_s2 to your robot's measured top speed
    // and acceleration headroom.
    const float direction = (turn_radians < 0.0f) ? -1.0f : 1.0f;
    const float total_dist = fabs(turn_radians);
    const float accel = accel_mm_s2 / robot.wheel_radius_mm;
    float peak_velocity = max_speed_mm_s / robot.wheel_radius_mm;
    float accel_time = peak_velocity / accel;
    float accel_dist = 0.5f * accel * accel_time * accel_time;
    if (2.0f * accel_dist > total_dist) {
        accel_dist = total_dist * 0.5f;
        accel_time = sqrt(accel_dist / (0.5f * accel));
        peak_velocity = accel * accel_time;
    }
    const float cruise_dist = total_dist - 2.0f * accel_dist;
    const float cruise_time = (peak_velocity > 0.001f) ? (cruise_dist / peak_velocity) : 0.0f;
    const float decel_start_time = accel_time + cruise_time;
    const float profile_duration = decel_start_time + accel_time;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small (near the end of
    // the profile), kp*error alone can drop below the motor's static
    // friction breakaway threshold and stall short of the target instead
    // of finishing. Tune to your motor's measured breakaway PWM.
    const int16_t min_effective_pwm = 25;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 2.5f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        const float elapsed = (now - start_time) * 1e-6f;
        float profile_dist;
        if (elapsed >= profile_duration) {
            profile_dist = total_dist;
        } else if (elapsed < accel_time) {
            profile_dist = 0.5f * accel * elapsed * elapsed;
        } else if (elapsed < decel_start_time) {
            profile_dist = accel_dist + peak_velocity * (elapsed - accel_time);
        } else {
            const float t_decel = elapsed - decel_start_time;
            profile_dist = accel_dist + cruise_dist + peak_velocity * t_decel - 0.5f * accel * t_decel * t_decel;
        }
        const float profile_pos = direction * profile_dist;

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(profile_pos - current_pos, dt, -100, 100);
        if (fabs(pos_error) > 0.2f && abs(forward_output) < min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
        }

        robot.gyroscope.update();
        float heading_error = 0.0f - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output = constrain(forward_output - heading_output, -120, 120);
        int16_t right_output = constrain(forward_output + heading_output, -120, 120);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


// Note that the distance argument is still technically just referring to displacement
void robot_drive_straight_with_lidars(
    const float distance,
    const unsigned long timeout_ms,
    bool stop_at_end = true,
    const float max_speed_mm_s = 500.0f,
    const float accel_mm_s2 = 800.0f
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(0.25f, 0.0f, 0.12f);
    // 1.2, 0.0, 0.05, 0.5 rn

    const float turn_radians = distance / robot.wheel_radius_mm;

    // Trapezoidal velocity profile over wheel-rotation radians: ramp up to
    // peak_velocity at `accel`, cruise, then ramp back down. Falls back to
    // a triangle profile (never reaching peak_velocity) on short moves.
    // Tune max_speed_mm_s / accel_mm_s2 to your robot's measured top speed
    // and acceleration headroom.
    const float direction = (turn_radians < 0.0f) ? -1.0f : 1.0f;
    const float total_dist = fabs(turn_radians);
    const float accel = accel_mm_s2 / robot.wheel_radius_mm;
    float peak_velocity = max_speed_mm_s / robot.wheel_radius_mm;
    float accel_time = peak_velocity / accel;
    float accel_dist = 0.5f * accel * accel_time * accel_time;
    if (2.0f * accel_dist > total_dist) {
        accel_dist = total_dist * 0.5f;
        accel_time = sqrt(accel_dist / (0.5f * accel));
        peak_velocity = accel * accel_time;
    }
    const float cruise_dist = total_dist - 2.0f * accel_dist;
    const float cruise_time = (peak_velocity > 0.001f) ? (cruise_dist / peak_velocity) : 0.0f;
    const float decel_start_time = accel_time + cruise_time;
    const float profile_duration = decel_start_time + accel_time;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small (near the end of
    // the profile), kp*error alone can drop below the motor's static
    // friction breakaway threshold and stall short of the target instead
    // of finishing. Tune to your motor's measured breakaway PWM.
    const int16_t min_effective_pwm = 60;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 2.5f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        const float elapsed = (now - start_time) * 1e-6f;
        float profile_dist;
        if (elapsed >= profile_duration) {
            profile_dist = total_dist;
        } else if (elapsed < accel_time) {
            profile_dist = 0.5f * accel * elapsed * elapsed;
        } else if (elapsed < decel_start_time) {
            profile_dist = accel_dist + peak_velocity * (elapsed - accel_time);
        } else {
            const float t_decel = elapsed - decel_start_time;
            profile_dist = accel_dist + cruise_dist + peak_velocity * t_decel - 0.5f * accel * t_decel * t_decel;
        }
        const float profile_pos = direction * profile_dist;

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(profile_pos - current_pos, dt, -150, 150);
        if (fabs(pos_error) > 0.2f && abs(forward_output) < min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        bool left_wall = left_dist < (180 - 88);
        bool right_wall = right_dist < (180 - 88);

        // wall_centering_controller outputs a heading bias in degrees
        float heading_target = 0.0f;
        float centering_tolerance_mm = 3.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else if (left_wall) {
            float lateral_error = left_dist - 52;
            if (fabs(lateral_error) < centering_tolerance_mm) { // mm, tune to your sensor noise floor
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else if (right_wall) {
            float lateral_error = 52 - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) { // mm, tune to your sensor noise floor
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else {
            wall_centering_controller.reset();
        }

        float heading_error = heading_target - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output  = constrain(forward_output - heading_output, -200, 200);
        int16_t right_output = constrain(forward_output + heading_output, -200, 200);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    if (stop_at_end) {
        robot_stop();
    }
}


void robot_drive_straight_with_lidars_no_profile(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(0.35f, 0.0f, 0.8f);
    // 1.2, 0.0, 0.05, 0.5 rn

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small, kp*error alone can
    // drop below the motor's static friction breakaway threshold and stall
    // short of the target instead of finishing. Tune to your motor's
    // measured breakaway PWM.
    const int16_t min_effective_pwm = 50;

    // Require a few consecutive close readings before trusting the front
    // wall as real - readFront() defaults to -1 (stale sentinel, always
    // "<=50") until the sensor's first real reading lands, and even once
    // it's reading for real, a single noisy sample shouldn't abort the
    // whole drive.
    // uint8_t front_wall_count = 0;
    // const uint8_t front_wall_debounce = 3;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 20.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        #if 1
        if (robot.lidar_system.readFront() <= 40) {
            break;
        }
        robot.lidar_system.update();
        #endif

        // if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 50) {
        //     front_wall_count++;
        //     if (front_wall_count >= front_wall_debounce) {
        //         robot.oled.clear();
        //         robot.oled.print(0, 0, "Braked");
        //         robot.oled.print(0, 1, "%d", (int) distance);
        //         break;
        //     }
        // } else {
        //     front_wall_count = 0;
        // }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -max_abs_pwm, max_abs_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        bool left_wall = left_dist < (180.0f - 88.0f);
        bool right_wall = right_dist < (180.0f - 88.0f);

        // wall_centering_controller outputs a heading bias in degrees
        float heading_target = 0.0f;
        float centering_tolerance_mm = 4.0f;
        /*if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
        } else */
        if (left_wall) {
            float lateral_error = left_dist - 55.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
        } else if (right_wall) {
            float lateral_error = 55.0f - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
        } else {
            wall_centering_controller.reset();
        }

        float heading_error = heading_target - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -50, 50);

        int16_t left_output  = constrain(forward_output - heading_output, -160, 160);
        int16_t right_output = constrain(forward_output + heading_output, -160, 160);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    if (stop_at_end) {
        robot_stop();
    }
}




// Same as robot_drive_straight_with_lidars_no_profile, but also watches
// for the robot having settled into genuinely straight, wall-parallel
// travel and re-zeroes the gyroscope heading reference at that moment -
// a cheap way to correct accumulated gyro drift mid-move using the walls
// as an independent, drift-free reference.
void robot_drive_straight_reset_gyroscope(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(0.5f, 0.0f, 0.2f);

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small, kp*error alone can
    // drop below the motor's static friction breakaway threshold and stall
    // short of the target instead of finishing. Tune to your motor's
    // measured breakaway PWM.
    const int16_t min_effective_pwm = 50;

    // "Settled straight" detection: require the lateral wall-centering
    // error to be small AND stable (not still actively converging), and
    // the robot's real heading to have actually caught up to the
    // commanded heading_target - not just that heading_target itself
    // happens to be near zero this instant. All three have to hold for
    // several consecutive iterations (not a single lucky sample) before
    // we trust it enough to re-zero the gyroscope. Tune the tolerances
    // and debounce count empirically.
    const float straight_lateral_tolerance_mm = 2.0f;
    const float straight_lateral_derivative_tolerance = 5.0f;
    const float straight_heading_tolerance_deg = 1.0f;
    const uint8_t straight_debounce = 10;
    float prev_lateral_error = 0.0f;
    uint8_t straight_count = 0;
    bool already_reset = false;

    // lateral_error only actually changes when a fresh lidar sample lands
    // (every ~20ms), but this loop runs much faster than that - a raw
    // instantaneous derivative would mostly read ~0 between samples, then
    // spike hugely the instant a new (even 1mm-different) sample arrives,
    // since that jump gets divided by the loop's much smaller dt rather
    // than the ~20ms it actually took. Smooth it the same way robot_rotate
    // smooths its own derivative, so single noisy samples can't spike it.
    float smoothed_lateral_derivative = 0.0f;
    const float lateral_derivative_smoothing = 0.2f;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 20.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -max_abs_pwm, max_abs_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        bool left_wall = left_dist < (180.0f - 88.0f);
        bool right_wall = right_dist < (180.0f - 88.0f);

        // wall_centering_controller outputs a heading bias in degrees
        float heading_target = 0.0f;
        float lateral_error = 0.0f;
        float centering_tolerance_mm = 1.0f;
        if (left_wall && right_wall) {
            lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else if (left_wall) {
            lateral_error = left_dist - 55.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else if (right_wall) {
            lateral_error = 55.0f - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -10, 10));
        } else {
            wall_centering_controller.reset();
        }

        float heading_error = heading_target - robot.gyroscope.getHeading();

        bool wall_detected = left_wall || right_wall;
        if (dt > 0.0f) {
            float raw_lateral_derivative = (lateral_error - prev_lateral_error) / dt;
            smoothed_lateral_derivative = smoothed_lateral_derivative * (1.0f - lateral_derivative_smoothing)
                                         + raw_lateral_derivative * lateral_derivative_smoothing;
        }
        prev_lateral_error = lateral_error;

        bool settled_straight = wall_detected
            && fabs(lateral_error) < straight_lateral_tolerance_mm
            && fabs(smoothed_lateral_derivative) < straight_lateral_derivative_tolerance
            && fabs(heading_error) < straight_heading_tolerance_deg;

        if (settled_straight) {
            if (straight_count < straight_debounce) {
                straight_count++;
            } else if (!already_reset) {
                robot.gyroscope.reset();
                already_reset = true;
            }
        } else {
            straight_count = 0;
            already_reset = false;
        }

        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -30, 30);

        int16_t left_output  = constrain(forward_output - heading_output, -160, 160);
        int16_t right_output = constrain(forward_output + heading_output, -160, 160);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    if (stop_at_end) {
        robot_stop();
    }
}


// Cubic (smoothstep) position profile: position(t) = D*(3s^2 - 2s^3) where
// s = t/T. Velocity rises and falls as a single smooth parabolic hump
// (zero at both ends) instead of the trapezoid's flat cruise plateau, and
// acceleration only jumps once at the very start/end instead of four
// times - lower-jerk, at the cost of a lower average speed for the same
// peak velocity/acceleration budget.
void robot_drive_straight_cubic_profile(
    const float distance,
    const unsigned long timeout_ms,
    bool stop_at_end = true,
    const float max_speed_mm_s = 1000.0f, // 800
    const float accel_mm_s2 = 600.0f // 1000
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.gyroscope.reset();
    mm::PIDController wall_centering_controller(0.25f, 0.0f, 0.12f);

    const float turn_radians = distance / robot.wheel_radius_mm;

    // Cubic profile duration: for position(t) = D*(3s^2-2s^3), peak
    // velocity is 1.5*D/T (at s=0.5) and peak |acceleration| is 6*D/T^2
    // (at s=0 and s=1). Pick whichever T satisfies both the peak-speed
    // and peak-acceleration budgets - same two tunables as the
    // trapezoidal profile functions, tune to your robot's measured top
    // speed and acceleration headroom.
    const float direction = (turn_radians < 0.0f) ? -1.0f : 1.0f;
    const float total_dist = fabs(turn_radians);
    const float peak_velocity_limit = max_speed_mm_s / robot.wheel_radius_mm;
    const float peak_accel_limit = accel_mm_s2 / robot.wheel_radius_mm;
    const float duration_for_speed = (peak_velocity_limit > 0.001f) ? (1.5f * total_dist / peak_velocity_limit) : 0.0f;
    const float duration_for_accel = (peak_accel_limit > 0.001f) ? sqrt(6.0f * total_dist / peak_accel_limit) : 0.0f;
    const float profile_duration = (duration_for_speed > duration_for_accel) ? duration_for_speed : duration_for_accel;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small (near the end of
    // the profile), kp*error alone can drop below the motor's static
    // friction breakaway threshold and stall short of the target instead
    // of finishing. Tune to your motor's measured breakaway PWM.
    const int16_t min_effective_pwm = 50;

    // Require a few consecutive close readings before trusting the front
    // wall as real - readFront() defaults to -1 (stale sentinel, always
    // "<=50") until the sensor's first real reading lands, and even once
    // it's reading for real, a single noisy sample shouldn't abort the
    // whole drive.
    uint8_t front_wall_count = 0;
    const uint8_t front_wall_debounce = 3;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 20.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 50) {
            front_wall_count++;
            if (front_wall_count >= front_wall_debounce) {
                break;
            }
        } else {
            front_wall_count = 0;
        }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        const float elapsed = (now - start_time) * 1e-6f;
        float s = (profile_duration > 0.0f) ? (elapsed / profile_duration) : 1.0f;
        s = constrain(s, 0.0f, 1.0f);
        const float profile_dist = total_dist * (3.0f * s * s - 2.0f * s * s * s);
        const float profile_pos = direction * profile_dist;

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(profile_pos - current_pos, dt, -140, 140);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        bool left_wall = left_dist < (180 - 88);
        bool right_wall = right_dist < (180 - 88);

        // wall_centering_controller outputs a heading bias in degrees
        float heading_target = 0.0f;
        float centering_tolerance_mm = 3.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else if (left_wall) {
            float lateral_error = left_dist - 52;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else if (right_wall) {
            float lateral_error = 52 - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -8, 8));
        } else {
            wall_centering_controller.reset();
        }

        float heading_error = heading_target - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output  = constrain(forward_output - heading_output, -180, 180);
        int16_t right_output = constrain(forward_output + heading_output, -180, 180);
        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    if (stop_at_end) {
        robot_stop();
    }
}


// HAVE TO DOUBLE CHECK THE DIRECTION OF DEGREES
void robot_turn(const float degrees, const float radius_mm, const unsigned long timeout_ms) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.gyroscope.reset();

    const float half_track_width = robot.track_width_mm / 2.0f;

    // left wheel should travel 2π(half_track_width + r)
    // right wheel should travel 2π(half_track_width - r) if degrees >= 0, and vice versa if degrees < 0
    const float left_to_right_ratio = (degrees >= 0.0f) ?
            (radius_mm - half_track_width) / (radius_mm + half_track_width) :
            (radius_mm + half_track_width) / (radius_mm - half_track_width);

    int16_t max_left_speed;
    int16_t max_right_speed;
    if (degrees >= 0.0f) {
        max_left_speed = 140;
        max_right_speed = static_cast<int16_t>(static_cast<float>(max_left_speed) / left_to_right_ratio);
    } else {
        max_right_speed = 140;
        max_left_speed = static_cast<int16_t>(static_cast<float>(max_right_speed) / left_to_right_ratio);
    }

    const float decel_start_deg = 25.0f;
    const float min_speed_scale = 0.3f;

    unsigned long start_time = micros();
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    bool turn_complete = false;

    while (!turn_complete && micros() - start_time < timeout) {
        robot.gyroscope.update();

        const float current_heading = fabs(robot.gyroscope.getHeading());
        const float target_heading = fabs(degrees);
        const float remaining = target_heading - current_heading;

        float speed_scale = 1.0f;
        if (remaining < decel_start_deg) {
            speed_scale = constrain(remaining / decel_start_deg, min_speed_scale, 1.0f);
        }

        robot.left_motor.setPWM(static_cast<int16_t>(max_left_speed * speed_scale));
        robot.right_motor.setPWM(static_cast<int16_t>(max_right_speed * speed_scale));

        turn_complete = (current_heading >= target_heading);
    }

    // Brief reverse pulse to cancel residual rotational momentum before
    // cutting power, rather than coasting to a stop.
    // robot.left_motor.setPWM(-max_left_speed / 2);
    // robot.right_motor.setPWM(-max_right_speed / 2);
    // delay(20); // tune empirically for your robot's mass/speed

    // robot.left_motor.setPWM(0);
    // robot.right_motor.setPWM(0);
}


void chaining(char *movement) {
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
        switch (movement[i]) {
            case 'f': {
                int n = count_forwards(&movement[i]);
                if (n > 2) {
                    mm::robot_drive_straight_with_lidars_no_profile((float) n * 180.0f, 5000 * n, 150); // to change
                    // mm::robot_drive_straight_cubic_profile((float) n * 180.0f, 5000 * n); // to change
                } else {
                    mm::robot_drive_straight_with_lidars_no_profile((float) n * 180.0f, 5000 * n, 150); // to change
                }
                i += n;
                break;
            } case 'r': {
                mm::robot_rotate(-90.0f, 600, 150);
                i++;
                break;
            } case 'l': {
                mm::robot_rotate(90.0f, 600, 150);
                i++;
                break;
            } default: {
                HALT();
            }
        }
    }

    // GET_ROBOT().oled.clear();
    // GET_ROBOT().oled.print(0, 0, "i: %d\nmovt[i]: %c", i, movement[i]);
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
            robot_rotate(movement[i] == 'l' ? 90.0f : -90.0f, 3000, 150);
            i++;
            continue;
        }

        int num_f = count_forwards(movement + i);

        if (movement[i + num_f] != 'l' && movement[i + num_f] != 'r') {
            // String ends
            robot_drive_straight_cubic_profile((float) num_f * CELL, 15000);
            // robot_drive_straight_with_lidars((float) num_f * CELL, 15000);
            break;
        }

        robot_drive_straight_cubic_profile(((float) num_f - 1.0f) * CELL + HALF_CELL, 15000);
        // robot_drive_straight_with_lidars(((float) num_f - 1.0f) * CELL + HALF_CELL, 15000);
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



// // Same as robot_drive_straight_with_lidars, but feeds the full remaining
// // distance straight to position_controller instead of a trapezoidal
// // setpoint ramp - no acceleration/deceleration shaping, matches the
// // original pre-profile behaviour.
// void robot_drive_straight_with_lidars_no_profile(
//     const float distance,
//     const unsigned long timeout_ms,
//     const int16_t max_abs_pwm,
//     bool stop_at_end = true
// ) {
//     auto& robot = GET_ROBOT();

//     robot.left_motor.setEncoderToZero();
//     robot.right_motor.setEncoderToZero();
//     robot.position_controller.reset();
//     robot.heading_controller.reset();
//     robot.gyroscope.reset();
//     mm::PIDController wall_centering_controller(0.3f, 0.0f, 0.08f);
//     // 1.2, 0.0, 0.05, 0.5 rn

//     const float turn_radians = distance / robot.wheel_radius_mm;

//     unsigned long start_time = micros();
//     unsigned long prev_time = start_time;
//     const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

//     float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
//     float pos_error = turn_radians - current_pos;
//     float prev_pos_error = pos_error;
//     float pos_derivative = 0.0f;

//     // Minimum-effective PWM: once the true remaining distance is still
//     // outside tolerance but the tracked error is small, kp*error alone can
//     // drop below the motor's static friction breakaway threshold and stall
//     // short of the target instead of finishing. Tune to your motor's
//     // measured breakaway PWM.
//     const int16_t min_effective_pwm = 50;

//     // Require a few consecutive close readings before trusting the front
//     // wall as real - readFront() defaults to -1 (stale sentinel, always
//     // "<=50") until the sensor's first real reading lands, and even once
//     // it's reading for real, a single noisy sample shouldn't abort the
//     // whole drive.
//     // uint8_t front_wall_count = 0;
//     // const uint8_t front_wall_debounce = 3;

//     // Continuously re-anchor the gyroscope's zero to the current heading
//     // while the robot is genuinely tracking the wall (lateral_error
//     // deadbanded to 0), instead of catching one "settled" instant and
//     // doing a single reset. wall_centering_controller is P-only, so
//     // lateral_error crossing zero doesn't mean the robot is actually
//     // parallel yet - it can be a fleeting crossing mid-oscillation. A
//     // one-shot reset caught at the wrong crossing goes stale for the rest
//     // of the wall, so when the wall disappears the robot reverts to
//     // whatever heading it "confirmed" - which was often just the original
//     // skew. Rebasing every good sample instead means whatever heading is
//     // live right before the wall disappears - the freshest possible
//     // estimate of "parallel" - is what's kept. rebase() is a pure
//     // software zero-shift (no I2C transaction), so it's cheap enough to
//     // call every tick.
//     uint8_t straight_count = 0;
//     const uint8_t straight_debounce = 3; // ignore a single noisy sample
//     const float straight_heading_tolerance_deg = 4.0f;

//     while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 20.0f) && micros() - start_time < timeout) {
//         robot.gyroscope.update();
//         robot.lidar_system.update();

//         #if 1
//         if (robot.lidar_system.readFront() <= 40) {
//             break;
//         }
//         robot.lidar_system.update();
//         #endif

//         // if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 50) {
//         //     front_wall_count++;
//         //     if (front_wall_count >= front_wall_debounce) {
//         //         robot.oled.clear();
//         //         robot.oled.print(0, 0, "Braked");
//         //         robot.oled.print(0, 1, "%d", (int) distance);
//         //         break;
//         //     }
//         // } else {
//         //     front_wall_count = 0;
//         // }

//         unsigned long now = micros();
//         float dt = (now - prev_time) * 1e-6f;
//         prev_time = now;

//         current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
//         pos_error = turn_radians - current_pos;
//         if (dt > 0.0f) {
//             pos_derivative = (pos_error - prev_pos_error) / dt;
//         }
//         prev_pos_error = pos_error;

//         int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -max_abs_pwm, max_abs_pwm);
//         if (fabs(pos_error) > 0.1f && abs(forward_output) < min_effective_pwm) {
//             forward_output = (pos_error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
//         }

//         float left_dist = static_cast<float>(robot.lidar_system.readLeft());
//         float right_dist = static_cast<float>(robot.lidar_system.readRight());
//         bool left_wall = left_dist < (180.0f - 88.0f);
//         bool right_wall = right_dist < (180.0f - 88.0f);

//         // wall_centering_controller outputs a heading bias in degrees
//         float heading_target = 0.0f;
//         float lateral_error = 0.0f;
//         float centering_tolerance_mm = 4.0f;
//         /*if (left_wall && right_wall) {
//             lateral_error = (left_dist - right_dist) / 2.0f;
//             if (fabs(lateral_error) < centering_tolerance_mm) {
//                 lateral_error = 0.0f;
//             }
//             heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
//         } else */
//         if (left_wall) {
//             lateral_error = left_dist - 55.0f;
//             if (fabs(lateral_error) < centering_tolerance_mm) {
//                 lateral_error = 0.0f;
//             }
//             heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
//         } else if (right_wall) {
//             lateral_error = 55.0f - right_dist;
//             if (fabs(lateral_error) < centering_tolerance_mm) {
//                 lateral_error = 0.0f;
//             }
//             heading_target = static_cast<float>(wall_centering_controller.compute_output(lateral_error, dt, -25, 25));
//         } else {
//             wall_centering_controller.reset();
//         }

//         float heading_error = heading_target - robot.gyroscope.getHeading();

//         bool wall_detected = left_wall || right_wall;
//         bool tracking_well = wall_detected
//             && lateral_error == 0.0f
//             && fabs(heading_error) < straight_heading_tolerance_deg;

//         if (tracking_well) {
//             if (straight_count < straight_debounce) {
//                 straight_count++;
//             } else {
//                 robot.gyroscope.rebase();
//                 // Reflect the fresh zero this same iteration instead of
//                 // waiting a tick, so heading_output below isn't computed
//                 // against the about-to-be-stale reference.
//                 heading_error = heading_target - robot.gyroscope.getHeading();
//             }
//         } else {
//             straight_count = 0;
//         }

//         int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -40, 40);

//         int16_t left_output  = constrain(forward_output - heading_output, -150, 150);
//         int16_t right_output = constrain(forward_output + heading_output, -150, 150);
//         robot.left_motor.setPWM(left_output);
//         robot.right_motor.setPWM(right_output);
//     }

//     if (stop_at_end) {
//         robot_stop();
//     }
// }
