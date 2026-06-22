#pragma once

#include <Arduino.h>
#include "Motor.hpp"
#include "Encoder.hpp"

namespace mtrn3100 {

/*
Two control modes:

    POSITION - Controls the robots displacement, the target is defined
    as the angular displacement of both wheels in radians. i.e.
    if the target is set to be 2 * PI, then the robot will ideally move
    2 * PI * wheel_radius metres forward.

    ROTATION - Controls the robots heading, the target is defined as the bearing of
    the robot in radians (+ = CCW).
*/
enum class PIDMode {
    POSITION,
    ROTATION
};


// ─── Shared PID state ────────────────────────────────────────────────────────

struct PIDState {
    float integral = 0.0f;
    float prev_error = 0.0f;
    float integral_limit = 100.0f;
};

static float pid_compute(PIDState& state, float kp, float ki, float kd, float error, float dt) {
    state.integral += error * dt;
    state.integral = constrain(state.integral, -state.integral_limit, state.integral_limit);
    float derivative = (error - state.prev_error) / dt;
    state.prev_error = error;
    return (kp * error) + (ki * state.integral) + (kd * derivative);
}


// ─── PIDController ───────────────────────────────────────────────────────────

class PIDController {
public:
    /*
    ROTATION mode constructor — point turn using differential steering.
        - left_motor / left_encoder:   Left wheel.
        - right_motor / right_encoder: Right wheel.
        - track_width: Distance (metres) between the two wheel contact patches.
        - kp, ki, kd: PID gains.

    Target is the desired robot heading change in radians.
      Positive = CCW (left turn), negative = CW (right turn).

    Call left_encoder.setEncoderToZero() and right_encoder.setEncoderToZero()
    before setting a new rotation target so that getRotation() starts from 0.
    */
    PIDController(Motor& left_motor, Encoder& left_encoder,
                  Motor& right_motor, Encoder& right_encoder,
                  float track_width, float wheel_radius, float kp, float ki, float kd)
        : mode{PIDMode::ROTATION},
          left_motor{left_motor}, left_encoder{left_encoder},
          right_motor{right_motor}, right_encoder{right_encoder},
          kp{kp}, ki{ki}, kd{kd}, track_width{track_width}, wheel_radius{wheel_radius} {}


    /*
    Sets the target setpoint.
        - POSITION mode: target wheel rotation in radians (signed).
        - ROTATION mode: target robot heading change in radians
                         (+ = CCW, - = CW).
    */
    void setTarget(float target) {
        this->target = target;
    }


    /*
    Call this at a fixed interval from loop(). dt is the interval in seconds.

    POSITION mode: drives the single motor toward the target wheel angle.

    ROTATION mode: drives left motor backward and right motor forward (or vice
    versa) to rotate the robot on the spot. Output magnitude is the same for
    both wheels; sign is mirrored so the robot pivots around its centre.

    Example (1kHz):
        const float DT = 0.001f;
        void loop() {
            static unsigned long prev = 0;
            unsigned long now = micros();
            if (now - prev >= 1000) {
                prev = now;
                pid.update(DT);
            }
        }
    */
    void update(float dt) {
        if (mode == PIDMode::POSITION) {
            float error = target - ((left_encoder.getRotation() + right_encoder.getRotation()) / 2.0f);
            float output = pid_compute(state, kp, ki, kd, error, dt);
            output = constrain(output, -255.0f, 255.0f);

            left_motor.setPWM(static_cast<int16_t>(output));
            right_motor.setPWM(static_cast<int16_t>(output));

        } else if (mode == PIDMode::ROTATION) {
            // Robot heading from differential wheel odometry:
            //   heading = (right_travel - left_travel) / track_width
            // where travel = wheel_radius * getRotation(), but since
            // wheel_radius cancels when both wheels are the same size and
            // the user supplies track_width in matching units, we fold it in:
            //   heading_rad = (right_rot - left_rot) * wheel_radius / track_width
            // If you don't know wheel_radius separately, supply
            // (wheel_radius / track_width) as a single tuned scalar via kp.
            // Here we keep it explicit so units are clear.
            float right_rot = right_encoder.getRotation();
            float left_rot = left_encoder.getRotation();
            float heading = (right_rot - left_rot) * wheel_radius / track_width;

            float error  = target - heading;
            float output = pid_compute(state, kp, ki, kd, error, dt);
            output = constrain(output, -255.0f, 255.0f);

            // Left wheel drives backward, right wheel drives forward (CCW turn)
            // Signs flip automatically with the error sign for CW turns.
            left_motor.setPWM(static_cast<int16_t>(-output));
            right_motor.setPWM(static_cast<int16_t>(output));
        }
    }


    /*
        Resets the integral accumulator and derivative state, as well as the wheel
        encoders. Always call this before issuing a new target.
    */
    void reset() {
        state.integral = 0.0f;
        state.prev_error = 0.0f;
        left_encoder.setEncoderToZero();
        right_encoder.setEncoderToZero();
    }


    void setGains(float kp, float ki, float kd) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }


    void setIntegralLimit(float limit) {
        state.integral_limit = limit;
    }

private:
    PIDMode mode;

    Motor& left_motor;
    Encoder& left_encoder;
    Motor& right_motor;
    Encoder& right_encoder;

    float kp, ki, kd;
    float track_width; // Units in metres
    float wheel_radius;  // Units in metres
    float target = 0.0f;

    PIDState state;
};

}


/*
#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {

        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // TODO: IMPLIMENT PID CONTROLLER
        integral = 0;
        derivative = 0;
        output = 0;

        prev_error = 0;

        return output;
    }

    // Function used to return the last calculated error.
    // The error is the difference between the desired position and current position.
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
    }

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;


};

}  // namespace mtrn3100
*/
