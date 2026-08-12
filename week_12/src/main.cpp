#include <Arduino.h>
#include "Maze.hpp"
#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"
#include "AutoMapping.hpp"

void setup() {
    // Serial.begin(9600);
    // Serial.println(BUILD_TIMESTAMP);
    Wire.begin();
    Wire.setWireTimeout(3000, true);

    auto& robot = GET_ROBOT();
    robot.oled.begin();

    robot.lidar_system.initAll();

    delay(500); // Give time for gyroscope to be still
    robot.gyroscope.begin();

    // robot.oled.print(0, 0, "Setup complete");
}

void loop() {
    // auto& r = GET_ROBOT();
    mm::do_auto_mapping();
    // mm::chaining((char *)"frffflfrffrflfffrf");
    // mm::robot_drive_straight_with_lidars_no_profile(1000.0f, 30000, 130);



    // unsigned long s = millis();
    // while (true) {
    //     r.lidar_system.update();
    //     int le = r.lidar_system.readLeft();
    //     int ri = r.lidar_system.readRight();
    //     int fr = r.lidar_system.readFront();

    //     if (millis() - s >= 1000) {
    //         r.oled.clear();
    //         r.oled.print(0, 0, "left: %d", le);
    //         r.oled.print(0, 1, "front: %d", fr);
    //         r.oled.print(0, 2, "right: %d", ri);

    //         s = millis();
    //     }
    // }



    HALT();
}
