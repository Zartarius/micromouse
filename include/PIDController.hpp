#pragma once

#include <Arduino.h>

namespace mm {

class PIDController {
public:
    // ki * integral_limit is the max the integral term can contribute -
    // keep that product well inside [lo, hi] passed to compute_output, or
    // the integral alone can saturate the output and cause overshoot.
    PIDController(float kp, float ki, float kd, float integral_limit = 100.0f);

    // Computes a PID output from the current error, bounded to [lo, hi].
    int16_t compute_output(float error, float dt, int16_t lo, int16_t hi);

    void reset(void);
    void tune(float kp, float ki, float kd);

private:
    struct PIDState {
        float integral = 0.0f;
        float prev_error = 0.0f;
    } state;

    float kp, ki, kd;
    float integral_limit;
};

}
