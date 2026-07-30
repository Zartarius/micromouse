#include <Arduino.h>
#include "Maze.hpp"
#include "Robot.hpp"
#include "Movement.hpp"

void setup() {
    Serial.begin(9600);
    // Serial.println(BUILD_TIMESTAMP);
    Wire.begin();

    auto& robot = GET_ROBOT();
    robot.oled.begin();
    // robot.oled.print(0, 0, "Testing OLED: %d", 1);

    robot.lidar_system.initAll();

    delay(500); // Give time for gyroscope to be still
    robot.gyroscope.begin();

    Serial.println(F("Setup complete!"));
}

// usage should be:
// chaining("lffrlrfr");       or any order of movements, should receive 8 movements
void chaining(char *movement) {
    while (*movement != '\0') {
        switch (*movement) {
            case 'f':
                mm::robot_drive_straight(176.0f, 5000);
                break;
            case 'r':
                mm::robot_rotate(-90.0f, 3000);
                break;
            case 'l':
                mm::robot_rotate(90.0f, 3000);
                break;
            default:
                while (true) ;
        }

        movement++;
    }
}


void loop() {
    // auto& r = ROBOT;

    // mm::delayWhileUpdating(20000);
    chaining((char *)F("fflfrflfrflffflfrfrffrflfffrfffrfrfflf"));

    while (true) ;
}




// using namespace mtrn3100;
// Maze maze;

// void setup()
// {
//     Serial.begin(9600);

//     delay(1000);  // give serial monitor time to connect

//     // Add test walls
//     maze.setWall(0,0,Wall::EAST,true);
//     maze.setWall(0,1,Wall::SOUTH,true);

//     maze.setWall(1,1,Wall::EAST,true);
//     maze.setWall(1,2,Wall::SOUTH,true);

//     maze.setWall(2,2,Wall::EAST,true);
//     maze.setWall(2,3,Wall::SOUTH,true);

//     maze.setWall(3,3,Wall::EAST,true);
//     maze.setWall(3,4,Wall::SOUTH,true);

//     maze.setWall(2,7,Wall::WEST,true);
//     maze.setWall(3,7,Wall::WEST,true);

//     maze.setWall(5,6,Wall::WEST,true);
//     maze.setWall(6,6,Wall::SOUTH,true);

//     maze.setWall(8,1,Wall::NORTH,true);
//     maze.setWall(8,2,Wall::NORTH,true);

//     maze.setWall(7,2,Wall::EAST,true);
//     maze.setWall(6,2,Wall::EAST,true);

//     // Print maze
//     maze.printMaze();
// }

// void loop()
// {

// }



// #include <iostream>
// #include "maze.hpp"

// using namespace mtrn3100;

// int main()
// {
//     Maze maze;

//     maze.setWall(0,0,Wall::EAST,true);
//     maze.setWall(0,1,Wall::SOUTH,true);

//     maze.printMaze();

//     return 0;
// }