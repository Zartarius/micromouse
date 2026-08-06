#include <Arduino.h>
#include "Robot.hpp"
#include "Miscellaneous.hpp"

void setup() {
    Serial.begin(9600);
    // Serial.println(BUILD_TIMESTAMP);
    Wire.begin();

    auto& robot = ROBOT;
    robot.oled.begin();
    // robot.oled.print(0, 0, "Testing OLED: %d", 1);

    robot.collectiveLidars.initAll();

    delay(500); // Give time for gyroscope to be still
    robot.gyroscope.begin();

    Serial.println(F("Setup complete!"));
}


void turning(void) {
    auto& robot = ROBOT;

    robot.gyroscope.reset();

    rotate(robot.rotation_controller, -90.0f, false);

    mm::delayWhileUpdating(4000);

    rotate(robot.rotation_controller, -90.0f, false);
}

void driving_and_stopping(void) {
    auto& robot = ROBOT;

    const float target = 100.0f; // mm from wall
    robot.position_controller.reset();
    robot.heading_controller.reset();

    unsigned long prev_time = micros();

    while (true) {
        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        float distance_error = (float)robot.collectiveLidars.readFront() - target;
        int16_t forward_output = robot.position_controller.compute_output(distance_error / 16.0f, dt, -80, 80);

        robot.gyroscope.update();
        float heading_error = 0.0f - robot.gyroscope.getHeading();
        int16_t heading_output = robot.heading_controller.compute_output(heading_error, dt, -40, 40);

        int16_t left_output = -forward_output + heading_output;
        int16_t right_output = forward_output + heading_output;
        left_output = constrain(left_output, -80, 80);
        right_output = constrain(right_output, -80, 80);

        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }
}


void straight(mm::PIDController& position_controller,
                mm::PIDController& heading_controller,
                float distance) {
    auto& robot = ROBOT;

    robot.left_motor.setEncoderToZero();
    robot.right_motor.setEncoderToZero();

    float wheel_turns = distance / 16.0f;
    position_controller.reset();
    heading_controller.reset();
    robot.gyroscope.reset();

    unsigned long start_time = micros();
    unsigned long prev_time = start_time;

    const unsigned long timeout = MILLISECONDS_TO_MICROSECONDS(30000);

    float pos_error = wheel_turns - ((-robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);
    float prev_pos_error = pos_error;
    float pos_derivative = 0.0f;

    while ((fabs(pos_error) > 0.2f || fabs(pos_derivative) > 2.5f) &&
            micros() - start_time < timeout) {

        unsigned long now = micros();
        float dt = (now - prev_time) * 1e-6f;
        prev_time = now;

        pos_error = wheel_turns - ((-robot.left_motor.getRotation() + robot.right_motor.getRotation()) / 2.0f);

        if (dt > 0.0f) {
            pos_derivative = (pos_error - prev_pos_error) / dt;
        }
        prev_pos_error = pos_error;

        int16_t forward_output = position_controller.compute_output(pos_error, dt, -80, 80);

        robot.gyroscope.update();
        float heading_error = 0.0f - robot.gyroscope.getHeading();
        int16_t heading_output = heading_controller.compute_output(heading_error, dt, -60, 60);

        int16_t left_output = -forward_output + heading_output; // maybe do - heading_output if this doesn't work
        int16_t right_output = forward_output + heading_output;
        left_output = constrain(left_output, -100, 100);
        right_output = constrain(right_output, -100, 100);

        robot.left_motor.setPWM(left_output);
        robot.right_motor.setPWM(right_output);
    }

    robot.left_motor.setPWM(0);
    robot.right_motor.setPWM(0);
}


// usage should be:
// chaining("lffrlrfr");       or any order of movements, should receive 8 movements
void chaining(char *movement) {
    auto& robot = ROBOT;

    while (*movement != '\0') {
        switch (*movement) {
            case 'f':
                straight(robot.position_controller, robot.heading_controller, 176.0f);
                break;
            case 'r':
                rotate(robot.rotation_controller, -90.0f);
                break;
            case 'l':
                rotate(robot.rotation_controller, 90.0f);
                break;
            default:
                while (true) ;
        }

        movement++;
    }
}


void loop() {
    // auto& r = ROBOT;
    // int left = r.collectiveLidars.readLeft();
    // delay(1000);
    // int right = r.collectiveLidars.readRight();

    // Serial.print("Left: "); Serial.println(left);
    // Serial.print("Right: "); Serial.println(right);

    // straight(ROBOT.position_controller, ROBOT.heading_controller, 1050.0f);
    // turning();
    //driveStraight(ROBOT.position_controller, ROBOT.heading_controller, 1000);
    // driving_and_stopping();
    // mm::delayWhileUpdating(20000);
    chaining((char *)F("fflfrflfrflffflfrfrffrflfffrfffrfrfflf"));
    // straight(r.position_controller, r.heading_controller, .0f);
    // rotate(r.rotation_controller, 180.0);

    while (true) ;
}