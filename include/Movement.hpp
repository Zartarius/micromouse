#pragma once

#include "Robot.hpp"

namespace mm {

void robot_stop(void);

void robot_rotate(const float degrees, const unsigned long timeout_ms, const int16_t max_abs_pwm);

// Drives forward `distance` mm using lidar-based wall centering and a
// front-wall stop, with a soft-start PWM ramp over accel_ramp_ms.
void robot_drive_straight_with_lidars_no_profile_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end,
    const unsigned long accel_ramp_ms
);

// Same as above but IMU-only: no lidars, no wall centering, no front stop -
// holds heading against whatever it was at the start of the move.
void robot_drive_straight_no_lidars_soft_start(
    const float distance,
    const unsigned long timeout_ms,
    const int16_t max_abs_pwm,
    bool stop_at_end = true,
    const unsigned long accel_ramp_ms = 150
);

// Sweeps +-sweep_deg around the current heading, using the front lidar to
// find and return to the heading that reads closest to the wall ahead -
// squares the robot up perpendicular to it.
void robot_align(const float sweep_deg = 4.0f, const int16_t max_abs_pwm = 20, const unsigned long timeout_ms = 4000);

// Runs a sequence of moves: 'f' = forward one cell (a run of 'f's chains
// into one longer move), 'r' = turn right 90, 'l' = turn left 90.
void chaining(const char *movement);

}
