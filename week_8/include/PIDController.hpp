#pragma once

#include <Arduino.h>

namespace mm {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp{kp}, ki{ki}, kd{kd} {}

    /*
    Computes a pid output given the state and parameters. Output is Bounded in [lo, hi].
    */
    int16_t compute_output(float error, float dt, int16_t lo, int16_t hi) {
        state.integral += error * dt;
        state.integral = constrain(state.integral, -state.integral_limit, state.integral_limit);
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
        State of PID control, includes current integral value, previous error and the max integral absolute value
    */
    struct PIDState {
        float integral = 0.0f;
        float prev_error = 0.0f;
        float integral_limit = 100.0f;
    };

    float kp, ki, kd;

    PIDState state;
};


}