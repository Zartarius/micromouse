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

    float movements[][2] = {
        {26.6f, 805.0f},
        {-26.6f, 0.0f}
    };

    robot.rotation_controller.tune(6.0f, 0.4f, 0.45f);

    robot_drive_straight_with_lidars_no_profile_soft_start(CELL_SIZE_MM, 5000, 80, true, 150UL);
    robot_rotate(90.0f, 1500, 80);
    robot_drive_straight_with_lidars_no_profile_soft_start(CELL_SIZE_MM, 5000, 80, true, 150UL);

    for (const auto& [rotation, distance] : movements) {
        robot_rotate(rotation, 1500, 80);
        (void)distance;
        robot_drive_straight_no_lidars_soft_start(distance, 30000, 80);
    }

    robot_drive_straight_with_lidars_no_profile_soft_start(CELL_SIZE_MM, 5000, 80, true, 150UL);
}


}