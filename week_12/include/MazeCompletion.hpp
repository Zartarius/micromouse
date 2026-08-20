#pragma once

#include "Robot.hpp"
#include "Movement.hpp"

namespace mm {

void do_maze_completion(void) {
    auto& robot = GET_ROBOT();
    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Task 4.1");
    delayWhileUpdating(1000UL);

    unsigned long start_time = millis();

    chaining("ffff");

    delayWhileUpdating(3460UL);

    unsigned long finish_time = millis();

    char time_str[10];
    dtostrf((finish_time - start_time) / 1000.0f, 0, 2, time_str);

    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Time:");
    robot.oled.printLarge(0, 3, "%ss", time_str);
}
}