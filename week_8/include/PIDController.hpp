#pragma once

#include <Arduino.h>

namespace mm {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp{kp}, ki{ki}, kd{} {}

    /*
    Sets the target setpoint, automatically resets controller.
    Target must be in the same units as the error.
    */
    void setTarget(float target) {
        reset();
        this->target = target;
    }

    /*
    Computes a pid output given the state and parameters
    */
    float compute_output(float error, float dt) {
        state.integral += error * dt;
        state.integral = constrain(state.integral, -state.integral_limit, state.integral_limit);
        float derivative = (error - state.prev_error) / dt;
        state.prev_error = error;
        return (kp * error) + (ki * state.integral) + (kd * derivative);
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
    float target = 0.0f;

    PIDState state{};

};


}