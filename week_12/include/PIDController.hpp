#pragma once

#include <Arduino.h>

namespace mm {

class PIDController {
public:
    // integral_limit bounds the accumulated integral itself, not the
    // output - so ki * integral_limit is the max the integral term can
    // ever contribute. Keep that product well inside [lo, hi] passed to
    // compute_output, or the integral alone can saturate the output and
    // take a while to unwind, causing overshoot/oscillation once error
    // crosses zero.
    PIDController(float kp, float ki, float kd, float integral_limit = 100.0f)
        : kp{kp}, ki{ki}, kd{kd}, integral_limit{integral_limit} {}

    /*
    Computes a pid output given the state and parameters. Output is Bounded in [lo, hi].
    */
    int16_t compute_output(float error, float dt, int16_t lo, int16_t hi) {
        state.integral += error * dt;
        state.integral = constrain(state.integral, -integral_limit, integral_limit);
        float derivative = (error - state.prev_error) / dt;
        state.prev_error = error;

        float output =  (kp * error) + (ki * state.integral) + (kd * derivative);
        output = constrain(output, static_cast<float>(lo), static_cast<float>(hi));

        return static_cast<int16_t>(output);
    }

    void reset(void) {
        state.integral = 0.0f;
        state.prev_error = 0.0f;
    }

private:
    /*
        State of PID control, includes current integral value and previous error
    */
    struct PIDState {
        float integral = 0.0f;
        float prev_error = 0.0f;
    };

    float kp, ki, kd;
    float integral_limit;

    PIDState state;
};


}