#include <Arduino.h>
#include "maze.hpp"

using namespace mtrn3100;
Maze maze;

void setup()
{
    Serial.begin(9600);

    delay(1000);  // give serial monitor time to connect

    // Add test walls
    maze.setWall(0,0,Wall::EAST,true);
    maze.setWall(0,1,Wall::SOUTH,true);

    maze.setWall(1,1,Wall::EAST,true);
    maze.setWall(1,2,Wall::SOUTH,true);

    maze.setWall(2,2,Wall::EAST,true);
    maze.setWall(2,3,Wall::SOUTH,true);

    maze.setWall(3,3,Wall::EAST,true);
    maze.setWall(3,4,Wall::SOUTH,true);

    maze.setWall(2,7,Wall::WEST,true);
    maze.setWall(3,7,Wall::WEST,true);

    maze.setWall(5,6,Wall::WEST,true);
    maze.setWall(6,6,Wall::SOUTH,true);

    maze.setWall(8,1,Wall::NORTH,true);
    maze.setWall(8,2,Wall::NORTH,true);

    maze.setWall(7,2,Wall::EAST,true);
    maze.setWall(6,2,Wall::EAST,true);

    // Print maze
    maze.printMaze();
}

void loop()
{

}

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