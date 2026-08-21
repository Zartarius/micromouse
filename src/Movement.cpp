#include "Movement.hpp"

#include "Misc.hpp"
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
            // Smoothed so a single noisy sample can't spike the derivative
            // and stall the exit condition.
            float raw_derivative = (error - prev_error) / dt;
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
    bool stop_at_end,
    const unsigned long accel_ramp_ms
) {
    auto& robot = GET_ROBOT();

    robot.lidar_system.updateFrontSmoothingAlpha(1.0f);
    robot.lidar_system.updateSideSmoothingAlpha(0.5f);

    robot.position_controller.tune(35.0f, 0.0f, 3.5f);

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.wall_centering_controller.reset();
    robot.gyroscope.reset();

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Floor below which kp*error can't overcome motor static friction, so
    // the robot stalls short of the target instead of finishing. Ramped
    // like max_abs_pwm so it can't exceed the current soft-start ceiling.
    const int16_t min_effective_pwm = 30;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 45) {
            break;
        }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Ramp the forward PWM ceiling from 0 to max_abs_pwm over
        // accel_ramp_ms, then hold - deceleration is left to the position PID.
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
        // hasReading() guard: the -1 "no reading yet" sentinel would
        // otherwise read as "wall at -1mm" and slam centering to its clamp.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // heading_target is a CCW-positive heading bias, so positive
        // lateral_error must mean "too close to the right wall".
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

        // Hard "too close" override: bypasses the PID ramp-up lag and snaps
        // heading_target to max authority away from a wall once it's
        // closer than critical_clearance_mm.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
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
            // Smoothed so a single noisy sample can't spike the derivative
            // and stall the exit condition.
            float raw_derivative = (error - prev_error) / dt;
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
    bool stop_at_end,
    const unsigned long accel_ramp_ms
) {
    auto& robot = GET_ROBOT();

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();
    robot.position_controller.reset();
    robot.heading_controller.reset();
    robot.wall_centering_controller.reset();
    robot.gyroscope.reset();

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Floor below which kp*error can't overcome motor static friction, so
    // the robot stalls short of the target instead of finishing. Ramped
    // like max_abs_pwm so it can't exceed the current soft-start ceiling.
    const int16_t min_effective_pwm = 30;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        if (robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 50) {
            break;
        }

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Ramp the forward PWM ceiling from 0 to max_abs_pwm over
        // accel_ramp_ms, then hold - deceleration is left to the position PID.
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
        // hasReading() guard: the -1 "no reading yet" sentinel would
        // otherwise read as "wall at -1mm" and slam centering to its clamp.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // heading_target is a CCW-positive heading bias, so positive
        // lateral_error must mean "too close to the right wall".
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

        // Hard "too close" override: bypasses the PID ramp-up lag and snaps
        // heading_target to max authority away from a wall once it's
        // closer than critical_clearance_mm.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
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
// gyroscope.reset(), i.e. straight).
void robot_drive_straight_no_lidars_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end,
    const unsigned long accel_ramp_ms
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

    // Floor below which kp*error can't overcome motor static friction.
    // Ramped like max_abs_pwm so it can't exceed the soft-start ceiling.
    const int16_t min_effective_pwm = 20;

    while ((fabs(pos_error) > 0.1f || fabs(pos_derivative) > 30.0f) && micros() - start_time < timeout) {
        robot.gyroscope.update();

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        // Ramp the forward PWM ceiling from 0 to max_abs_pwm over
        // accel_ramp_ms, then hold - deceleration is left to the position PID.
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

        // IMU-only heading hold: steer back to the heading recorded at the
        // start of the move (0 right after reset()).
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
            // Smoothed so a single noisy sample can't spike the derivative
            // and stall the exit condition.
            float raw_derivative = (error - prev_error) / dt;
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
    bool stop_at_end,
    const unsigned long accel_ramp_ms
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

    const float turn_radians = distance / robot.wheel_radius_mm;

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;
    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms);

    float current_pos = (robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f;
    float pos_error = turn_radians - current_pos;
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    // Floor below which kp*error can't overcome motor static friction, so
    // the robot stalls short of the target instead of finishing. Ramped
    // like max_abs_pwm so it can't exceed the current soft-start ceiling.
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

        // Ramp the forward PWM ceiling from 0 to max_abs_pwm over
        // accel_ramp_ms, then hold - deceleration is left to the position PID.
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
        // hasReading() guard: the -1 "no reading yet" sentinel would
        // otherwise read as "wall at -1mm" and slam centering to its clamp.
        bool left_wall = robot.lidar_system.leftHasReading() && left_dist < (CELL_SIZE_MM - 88.0f);
        bool right_wall = robot.lidar_system.rightHasReading() && right_dist < (CELL_SIZE_MM - 88.0f);

        // heading_target is a CCW-positive heading bias, so positive
        // lateral_error must mean "too close to the right wall".
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

        // Hard "too close" override: bypasses the PID ramp-up lag and snaps
        // heading_target to max authority away from a wall once it's
        // closer than critical_clearance_mm.
        const float critical_clearance_mm = 30.0f;
        bool left_critical = robot.lidar_system.leftHasReading() && left_dist < critical_clearance_mm;
        bool right_critical = robot.lidar_system.rightHasReading() && right_dist < critical_clearance_mm;
        if (left_critical || right_critical) {
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


// Sweeps +-sweep_deg around the current heading at low PWM, using the front
// lidar to find which heading reads closest to the wall ahead, then returns
// to it. Doesn't reuse robot_rotate() since that re-zeros the gyroscope on
// every call - this needs one consistent zero across all three legs.
void robot_align(const float sweep_deg, const int16_t max_abs_pwm, const unsigned long timeout_ms) {
    auto& robot = GET_ROBOT();

    robot.gyroscope.reset();

    // Each leg gets its own slice of the budget instead of sharing one
    // clock, so a single stalled leg can't eat the whole timeout.
    const unsigned long leg_timeout = MILLISECONDS_TO_MICROSECONDS(timeout_ms) / 3;

    float best_heading = 0.0f;
    int best_distance = 32767;

    // Same floor as min_effective_pwm in the drive functions: at low
    // ceilings like max_abs_pwm=20, kp*error alone can drop below static
    // friction and stall a few degrees short forever.
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
                robot_drive_straight_with_lidars_no_profile_soft_start((float) n * CELL_SIZE_MM, 3000 * n, 220, true, 120);
                i += n;
                break;
            } case 'r': {
                robot_rotate(-90.0f, 710, 150);
                i++;
                break;
            } case 'l': {
                robot_rotate(90.0f, 710, 150);
                i++;
                break;
            } default: {
                HALT();
            }
        }
    }
}


}
