#include "MazeCompletion.hpp"

#include "Task.hpp"

#if TASK_4_POINT == 1

#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"

namespace mm {

void do_maze_completion(void) {
    auto& robot = GET_ROBOT();
    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Task 4.1");
    delayWhileUpdating(1000UL);

    unsigned long start_time = millis();

    chaining("rrflflfrflffrflffflflfrflfffrfffrflfrfffrfrflfflflfrfrfflflffflfrfflfrflflfrffrflflfrflfrflfflf");

    unsigned long finish_time = millis();

    char time_str[10];
    dtostrf((finish_time - start_time) / 1000.0f, 0, 2, time_str);

    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Time:");
    robot.oled.printLarge(0, 3, "%ss", time_str);
}

}

#endif
