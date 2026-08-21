#pragma once

#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"

namespace mm {

void do_cont_planning(void) {
    auto& robot = GET_ROBOT();
    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Task 4.2");
    delayWhileUpdating(1000UL);

    robot.rotation_controller.tune(6.0f, 0.4f, 0.45f);

    float first_half[][2] = {{0.0, 180.0}, {0.0, 180.0}, {-90.0, 180.0}, {90.0, 180.0}, {0.0, 180.0}, {0.0, 180.0},
    {90.0, 180.0}, {90.0, 180.0}, {-90.0, 180.0}, {90.0, 180.0}, {0.0, 0.0}};

    float continuous[][2] = {{-52.8, 329.0}, {37.6, 95.3}, {19.1, 279.6}, {-26.9, 99.9}, {-61.0, 104.6}, {-28.3, 298.2}, {22.3, 0.0}};

    float second_half[][2] = {{0.0, 180.0}, {-90.0, 180.0}, {90.0, 180.0}, {-90.0, 180.0},
    {-90.0, 180.0}, {90.0, 180.0}, {0.0, 180.0}, {0.0, 0.0}};


    // make sure to use proper timeout
    for (const auto& [rotation, distance] : first_half) {
        robot_rotate(rotation, 5000, 80);
        robot_drive_straight_with_lidars_no_profile_soft_start(distance, 30000, 100, true, 200UL);
    }

    for (const auto& [rotation, distance] : continuous) {
        robot_rotate(rotation, 3000, 80);
        robot_drive_straight_no_lidars_soft_start(distance, 30000, 80, true, 300UL);
    }

    for (const auto& [rotation, distance] : second_half) {
        robot_rotate(rotation, 5000, 80);
        robot_drive_straight_with_lidars_no_profile_soft_start(distance, 30000, 100, true, 200UL);
    }
}


}