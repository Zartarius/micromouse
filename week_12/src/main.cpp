#include <Arduino.h>
#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"
#include "AutoMapping.hpp"

#define TASK_4_POINT 3

void setup() {
    // Serial.begin(9600);
    Wire.begin();
    Wire.setWireTimeout(3000, true);

    auto& robot = GET_ROBOT();
    robot.oled.begin();

    robot.lidar_system.initAll();

    delay(500); // Give time for gyroscope to be still
    robot.gyroscope.begin();
}

void loop() {
    // auto& r = GET_ROBOT();
    // mm::do_auto_mapping();
    // mm::chaining((char *)"frffflfrffrflfffrf");
    mm::robot_drive_straight_with_lidars_no_profile_soft_start(1000.0f, 30000, 130);
    // mm::print_victory_flag();

    HALT();
}