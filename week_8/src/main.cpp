#include <Arduino.h>
#include "Robot.hpp"
#include "Miscellaneous.hpp"

void setup() {
    Serial.begin(9600);
    // Serial.println(BUILD_TIMESTAMP);
    Wire.begin();

    auto& robot = ROBOT;
    // robot.oled.begin();
    // robot.oled.print("Testing OLED: %d %d %d", 1, 2, 3);

    robot.collectiveLidars.initAll();

    delay(500); // Give time for gyroscope to be still
    robot.gyroscope.begin();

    Serial.println("Setup complete!");
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

// usage should be:
// chaining("lffrlrfr");       or any order of movements, should receive 8 movements
void chaining(char *movement) {
    auto& robot = ROBOT;

    while (*movement != '\0') {
        if (*movement == 'f') {
            driveStraight(robot.position_controller, robot.heading_controller, 215.0f);
            //200 before, changed to 250 to make it go straight for longer
        } else if (*movement == 'r') {
            rotate(robot.rotation_controller, -90.0f);
        } else if (*movement == 'l') {
            rotate(robot.rotation_controller, 90.0f);
        } else {
            Serial.println("BAD INSTRUCTION!!!");
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

    // turning();
    driveStraight(ROBOT.position_controller, ROBOT.heading_controller, 1000);
    // driving_and_stopping();
    // mm::delayWhileUpdating(20000);
    // chaining((char *)"flffrfl");
}