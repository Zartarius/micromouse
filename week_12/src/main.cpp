#include <Arduino.h>
#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"
#include "Task.hpp"

#if TASK_4_POINT == 1
#include "MazeCompletion.hpp"
#elif TASK_4_POINT == 2
#include "ContPlanning.hpp"
#elif TASK_4_POINT == 3
#include "AutoMapping.hpp"
#else
#error "Invalid task number!"
#endif

void setup() {
    // Serial.begin(9600);
    Wire.begin();
    Wire.setWireTimeout(3000, true);

    auto& robot = GET_ROBOT();
    robot.oled.begin();

    robot.lidar_system.initAll();

    delay(300); // Give time for gyroscope to be still
    robot.gyroscope.begin();
}

void loop() {
    // auto& r = GET_ROBOT();
    // mm::robot_rotate(90.0f, 1100, 90);
    // mm::chaining((char *)"ffff");
    // mm::chaining((char *)"flfrffffrflfrflfrfrflffrflflfrfrffflfrfrflff");
    // mm::lidar_tester();
    // mm::robot_drive_straight_with_lidars_no_profile_soft_start(180.0f * 4.0f, 30000, 110);
    // mm::lidar_t ester();

    #if TASK_4_POINT == 1
        mm::do_maze_completion();
    #elif TASK_4_POINT == 2
        mm::do_cont_planning();
    #elif TASK_4_POINT == 3
        mm::do_auto_mapping();
    #endif

    HALT();
}