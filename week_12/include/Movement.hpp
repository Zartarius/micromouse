#pragma once

#include "Robot.hpp"
#include "Misc.hpp"
#include "PIDController.hpp"
#include "Task.hpp"

namespace mm {

void robot_stop(void) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


#if TASK_4_POINT == 1

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

    while ((fabs(error) > 2.0f || fabs(error_derivative) > 20.0f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        robot.gyroscope.update();
        robot.lidar_system.update();
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


void robot_drive_straight_with_lidars_no_profile_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true,
    const unsigned long accel_ramp_ms = 120
) {
    auto& robot = GET_ROBOT();

    robot.lidar_system.updateFrontSmoothingAlpha(1.0f);
    robot.lidar_system.updateSideSmoothingAlpha(0.3f);

    robot.position_controller.tune(45.0f, 0.0f, 2.8f);

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.wall_centering_controller.reset();
    robot.gyroscope.reset();
    // mm::PIDController wall_centering_controller(0.5f, 0.0f, 0.02f);
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
    // measured breakaway PWM. Ramped the same way as max_abs_pwm below, so
    // it can't force more torque than the soft-start currently allows.
    const int16_t min_effective_pwm = 30;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 65) {
            break;
        }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Fast soft-start: ramp the forward PWM ceiling from 0 up to
        // max_abs_pwm over accel_ramp_ms, then hold at max_abs_pwm for the
        // rest of the move - including the deceleration phase, which is
        // left entirely to the position PID.
        const float elapsed_ms = (now - start_time) * 1e-3f;
        const float ramp_scale = (accel_ramp_ms > 0)
            ? constrain(elapsed_ms / (float) accel_ramp_ms, 0.0f, 1.0f)
            : 1.0f;
        const int16_t ramped_max_pwm = (int16_t) (max_abs_pwm * ramp_scale);
        const int16_t ramped_min_effective_pwm = min(min_effective_pwm, ramped_max_pwm);

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -ramped_max_pwm, ramped_max_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < ramped_min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? ramped_min_effective_pwm : -ramped_min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        // -1 sentinel guard: without HasReading(), "no reading yet" reads
        // as "wall at -1mm" and slams wall-centering to its clamp before
        // either sensor has actually sampled anything.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // wall_centering_controller outputs a heading bias in degrees.
        // Sign convention: positive heading_target turns the robot CCW
        // (left, matching getHeading()'s CW-negative/CCW-positive and
        // robot_rotate's degrees>0=left), so positive lateral_error must
        // mean "too close to the right wall / too far from the left one".
        float heading_target = 0.0f;
        float centering_tolerance_mm = 4.0f;
        const float wall_centering_clamp = 12.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (left_wall) {
            float lateral_error = left_dist - 50.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (right_wall) {
            float lateral_error = 50.0f - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else {
            robot.wall_centering_controller.reset();
        }

        // Hard "too close" override: a smooth PID has to accumulate error
        // before it commands full authority, but a heading change only
        // becomes a lateral position change after some travel distance -
        // that lag is exactly what lets the robot graze a wall even though
        // the controller is "correct". Once either side crosses this
        // clearance, skip the ramp and snap heading_target straight to max
        // authority away from that wall. 25mm is a placeholder - measure
        // the real number by hand (push the chassis sideways into a wall
        // and read what the sensor reports at first contact) and set this
        // a few mm above that, not at 0.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
            // If somehow both trip at once, steer away from whichever is nearer.
            bool steer_away_from_left = left_critical && (!right_critical || left_dist <= right_dist);
            heading_target = steer_away_from_left ? -wall_centering_clamp : wall_centering_clamp;
            robot.wall_centering_controller.reset();
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

#elif TASK_4_POINT == 2

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
        robot.lidar_system.update();
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


// Same as robot_drive_straight_with_lidars_no_profile, but ramps the forward
// PWM ceiling from 0 up to max_abs_pwm over a short accel_ramp_ms window at
// the start of the move instead of allowing full torque immediately - a
// quick soft-start to cut the initial jerk. Deceleration is untouched: it's
// still whatever the position PID naturally does as pos_error shrinks, and
// by the time that happens the ramp has long since reached max_abs_pwm
// (keep accel_ramp_ms small so this holds).
void robot_drive_straight_with_lidars_no_profile_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true,
    const unsigned long accel_ramp_ms = 100
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.wall_centering_controller.reset();
    robot.gyroscope.reset();
    // mm::PIDController wall_centering_controller(0.5f, 0.0f, 0.02f);
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
    // measured breakaway PWM. Ramped the same way as max_abs_pwm below, so
    // it can't force more torque than the soft-start currently allows.
    const int16_t min_effective_pwm = 30;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        #if 1
        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 50) {
            // robot.oled.clear();
            // robot.oled.print(0, 0, "d: %d\n", d);
            // HALT();
            break;
        }
        #endif

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Fast soft-start: ramp the forward PWM ceiling from 0 up to
        // max_abs_pwm over accel_ramp_ms, then hold at max_abs_pwm for the
        // rest of the move - including the deceleration phase, which is
        // left entirely to the position PID.
        const float elapsed_ms = (now - start_time) * 1e-3f;
        const float ramp_scale = (accel_ramp_ms > 0)
            ? constrain(elapsed_ms / (float) accel_ramp_ms, 0.0f, 1.0f)
            : 1.0f;
        const int16_t ramped_max_pwm = (int16_t) (max_abs_pwm * ramp_scale);
        const int16_t ramped_min_effective_pwm = min(min_effective_pwm, ramped_max_pwm);

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -ramped_max_pwm, ramped_max_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < ramped_min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? ramped_min_effective_pwm : -ramped_min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        // -1 sentinel guard: without HasReading(), "no reading yet" reads
        // as "wall at -1mm" and slams wall-centering to its clamp before
        // either sensor has actually sampled anything.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // wall_centering_controller outputs a heading bias in degrees.
        // Sign convention: positive heading_target turns the robot CCW
        // (left, matching getHeading()'s CW-negative/CCW-positive and
        // robot_rotate's degrees>0=left), so positive lateral_error must
        // mean "too close to the right wall / too far from the left one".
        float heading_target = 0.0f;
        float centering_tolerance_mm = 4.0f;
        const float wall_centering_clamp = 12.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (left_wall) {
            float lateral_error = left_dist - 50.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (right_wall) {
            float lateral_error = 50.0f - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else {
            robot.wall_centering_controller.reset();
        }

        // Hard "too close" override: a smooth PID has to accumulate error
        // before it commands full authority, but a heading change only
        // becomes a lateral position change after some travel distance -
        // that lag is exactly what lets the robot graze a wall even though
        // the controller is "correct". Once either side crosses this
        // clearance, skip the ramp and snap heading_target straight to max
        // authority away from that wall. 25mm is a placeholder - measure
        // the real number by hand (push the chassis sideways into a wall
        // and read what the sensor reports at first contact) and set this
        // a few mm above that, not at 0.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
            // If somehow both trip at once, steer away from whichever is nearer.
            bool steer_away_from_left = left_critical && (!right_critical || left_dist <= right_dist);
            heading_target = steer_away_from_left ? -wall_centering_clamp : wall_centering_clamp;
            robot.wall_centering_controller.reset();
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


// Same as robot_drive_straight_with_lidars_no_profile_soft_start, but skips
// the lidars entirely - no wall-centering, no front-wall stop - and holds
// heading with the IMU alone (steers back to the heading recorded at
// gyroscope.reset(), i.e. straight). Soft-start ramp and position-PID logic
// are otherwise identical.
void robot_drive_straight_no_lidars_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true,
    const unsigned long accel_ramp_ms = 150
) {
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

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Minimum-effective PWM: once the true remaining distance is still
    // outside tolerance but the tracked error is small, kp*error alone can
    // drop below the motor's static friction breakaway threshold and stall
    // short of the target instead of finishing. Tune to your motor's
    // measured breakaway PWM. Ramped the same way as max_abs_pwm below, so
    // it can't force more torque than the soft-start currently allows.
    const int16_t min_effective_pwm = 20;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Fast soft-start: ramp the forward PWM ceiling from 0 up to
        // max_abs_pwm over accel_ramp_ms, then hold at max_abs_pwm for the
        // rest of the move - including the deceleration phase, which is
        // left entirely to the position PID.
        const float elapsed_ms = (now - start_time) * 1e-3f;
        const float ramp_scale = (accel_ramp_ms > 0)
            ? constrain(elapsed_ms / (float) accel_ramp_ms, 0.0f, 1.0f)
            : 1.0f;
        const int16_t ramped_max_pwm = (int16_t) (max_abs_pwm * ramp_scale);
        const int16_t ramped_min_effective_pwm = min(min_effective_pwm, ramped_max_pwm);

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -ramped_max_pwm, ramped_max_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < ramped_min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? ramped_min_effective_pwm : -ramped_min_effective_pwm;
        }

        // IMU-only heading hold: no lidars, so just steer back to the
        // heading recorded at the start of the move (0 right after reset()).
        float heading_error = 0.0f - robot.gyroscope.getHeading();
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

#elif TASK_4_POINT == 3

void robot_rotate(const float degrees, const unsigned long timeout_ms, const int16_t max_abs_pwm) {
    auto& robot = GET_ROBOT();

    robot.rotation_controller.tune(6.3f, 0.06f, 0.6f);

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

    while ((fabs(error) > 0.5f || fabs(error_derivative) > 15.0f) && micros() - start_time < timeout) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        robot.gyroscope.update();
        robot.lidar_system.update();
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


void robot_drive_straight_with_lidars_no_profile_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true,
    const unsigned long accel_ramp_ms = 200
) {
    auto& robot = GET_ROBOT();

    robot.lidar_system.updateFrontSmoothingAlpha(0.8f);
    robot.lidar_system.updateSideSmoothingAlpha(0.25f);

    robot.wall_centering_controller.tune(0.20f, 0.0f, 0.0f);

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.wall_centering_controller.reset();
    robot.gyroscope.reset();
    // mm::PIDController wall_centering_controller(0.5f, 0.0f, 0.02f);
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
    // measured breakaway PWM. Ramped the same way as max_abs_pwm below, so
    // it can't force more torque than the soft-start currently allows.
    const int16_t min_effective_pwm = 30;

    while ((fabs(pos_error) > 0.15f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 55) {
            break;
        }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Fast soft-start: ramp the forward PWM ceiling from 0 up to
        // max_abs_pwm over accel_ramp_ms, then hold at max_abs_pwm for the
        // rest of the move - including the deceleration phase, which is
        // left entirely to the position PID.
        const float elapsed_ms = (now - start_time) * 1e-3f;
        const float ramp_scale = (accel_ramp_ms > 0)
            ? constrain(elapsed_ms / (float) accel_ramp_ms, 0.0f, 1.0f)
            : 1.0f;
        const int16_t ramped_max_pwm = (int16_t) (max_abs_pwm * ramp_scale);
        const int16_t ramped_min_effective_pwm = min(min_effective_pwm, ramped_max_pwm);

        current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
        pos_error = turn_radians - current_pos;
        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = robot.position_controller.compute_output(pos_error, dt, -ramped_max_pwm, ramped_max_pwm);
        if (fabs(pos_error) > 0.1f && abs(forward_output) < ramped_min_effective_pwm) {
            forward_output = (pos_error > 0.0f) ? ramped_min_effective_pwm : -ramped_min_effective_pwm;
        }

        float left_dist = static_cast<float>(robot.lidar_system.readLeft());
        float right_dist = static_cast<float>(robot.lidar_system.readRight());
        // -1 sentinel guard: without HasReading(), "no reading yet" reads
        // as "wall at -1mm" and slams wall-centering to its clamp before
        // either sensor has actually sampled anything.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // wall_centering_controller outputs a heading bias in degrees.
        // Sign convention: positive heading_target turns the robot CCW
        // (left, matching getHeading()'s CW-negative/CCW-positive and
        // robot_rotate's degrees>0=left), so positive lateral_error must
        // mean "too close to the right wall / too far from the left one".
        float heading_target = 0.0f;
        float centering_tolerance_mm = 4.0f;
        const float wall_centering_clamp = 12.0f;
        if (left_wall && right_wall) {
            float lateral_error = (left_dist - right_dist) / 2.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (left_wall) {
            float lateral_error = left_dist - 50.0f;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else if (right_wall) {
            float lateral_error = 50.0f - right_dist;
            if (fabs(lateral_error) < centering_tolerance_mm) {
                lateral_error = 0.0f;
            }
            heading_target = static_cast<float>(robot.wall_centering_controller.compute_output(lateral_error, dt, -wall_centering_clamp, wall_centering_clamp));
        } else {
            robot.wall_centering_controller.reset();
        }

        // Hard "too close" override: a smooth PID has to accumulate error
        // before it commands full authority, but a heading change only
        // becomes a lateral position change after some travel distance -
        // that lag is exactly what lets the robot graze a wall even though
        // the controller is "correct". Once either side crosses this
        // clearance, skip the ramp and snap heading_target straight to max
        // authority away from that wall. 25mm is a placeholder - measure
        // the real number by hand (push the chassis sideways into a wall
        // and read what the sensor reports at first contact) and set this
        // a few mm above that, not at 0.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
            // If somehow both trip at once, steer away from whichever is nearer.
            bool steer_away_from_left = left_critical && (!right_critical || left_dist <= right_dist);
            heading_target = steer_away_from_left ? -wall_centering_clamp : wall_centering_clamp;
            robot.wall_centering_controller.reset();
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

#endif


// Sweeps +-sweep_deg around the current heading at low PWM, using the
// front lidar to find which heading in that range reads closest to the
// wall ahead, then returns to that heading - the closest point across a
// sweep is where the sensor is most head-on to the wall, so this squares
// the robot up perpendicular to it.
//
// Doesn't reuse robot_rotate() here because that resets the gyroscope
// zero point on every call - this function needs one consistent zero
// across all three legs so best_heading means the same thing throughout.
void robot_align(const float sweep_deg = 4.0f, const int16_t max_abs_pwm = 20, const unsigned long timeout_ms = 4000) {
    auto& robot = GET_ROBOT();

    robot.gyroscope.reset();
    // robot.lidar_system.updateSmoothingAlpha(0.9f);

    // Each leg gets its own slice of the budget instead of sharing one
    // clock - otherwise a single stalled leg (see min_effective_pwm below)
    // eats the whole timeout and the remaining legs never run at all,
    // which is exactly what was happening.
    const unsigned long leg_timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms) / 3;

    float best_heading = 0.0f;
    int best_distance = 32767;

    // Same idea as the min_effective_pwm floor in the drive functions:
    // kp*error tapers as the robot nears target_deg, and at low ceilings
    // like max_abs_pwm=40 it can drop below what's needed to overcome
    // static friction before error is inside tolerance - the robot then
    // just stalls a few degrees short forever. Tune to your motor's
    // measured rotational breakaway PWM.
    const int16_t min_effective_pwm = 20;

    auto rotate_and_sample = [&](float target_deg) {
        robot.rotation_controller.reset();
        unsigned long leg_start = micros();
        unsigned long prev_time = leg_start;
        float error = target_deg - robot.gyroscope.getHeading();

        while (fabs(error) > 0.5f && micros() - leg_start < leg_timeout) {
            unsigned long now = micros();
            float dt = (now - prev_time) * 1e-6f;
            prev_time = now;

            robot.gyroscope.update();
            robot.lidar_system.update();
            error = target_deg - robot.gyroscope.getHeading();

            if (robot.lidar_system.frontHasReading()) {
                int d = robot.lidar_system.readFront();
                if (d < best_distance) {
                    best_distance = d;
                    best_heading = robot.gyroscope.getHeading();
                }
            }

            int16_t output = robot.rotation_controller.compute_output(error, dt, -max_abs_pwm, max_abs_pwm);
            if (fabs(error) > 1.0f && abs(output) < min_effective_pwm) {
                output = (error > 0.0f) ? min_effective_pwm : -min_effective_pwm;
            }
            robot.left_motor.setPWM(-output);
            robot.right_motor.setPWM(output);
        }
    };

    rotate_and_sample(-sweep_deg);
    rotate_and_sample(sweep_deg);
    rotate_and_sample(best_heading);

    robot_stop();
}


void chaining(const char *movement) {
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
                mm::robot_drive_straight_with_lidars_no_profile_soft_start((float) n * CELL_SIZE_MM, 5000 * n, 130);
                i += n;
                break;
            } case 'r': {
                mm::robot_rotate(-90.0f, 700, 140);
                i++;
                break;
            } case 'l': {
                mm::robot_rotate(90.0f, 700, 140);
                i++;
                break;
            } default: {
                HALT();
            }
        }
    }
}


}
