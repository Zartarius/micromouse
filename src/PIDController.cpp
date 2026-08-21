#include "PIDController.hpp"

namespace mm {

PIDController::PIDController(float kp, float ki, float kd, float integral_limit)
    : kp{kp}, ki{ki}, kd{kd}, integral_limit{integral_limit} {}

int16_t PIDController::compute_output(float error, float dt, int16_t lo, int16_t hi) {
    state.integral += error * dt;
    state.integral = constrain(state.integral, -integral_limit, integral_limit);
    float derivative = (error - state.prev_error) / dt;
    state.prev_error = error;

    float output = (kp * error) + (ki * state.integral) + (kd * derivative);
    output = constrain(output, static_cast<float>(lo), static_cast<float>(hi));

    return static_cast<int16_t>(output);
}

void PIDController::reset(void) {
    state.integral = 0.0f;
    state.prev_error = 0.0f;
}

void PIDController::tune(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

}
