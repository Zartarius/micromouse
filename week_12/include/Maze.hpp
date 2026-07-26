#pragma once
#include <Arduino.h>
namespace mtrn3100
{

constexpr int MAZE_SIZE = 9;

enum class Wall
{
    NORTH = 0,
    EAST,
    SOUTH,
    WEST
};

struct Cell
{
    bool walls[4] = {false, false, false, false};
    bool visited = false;
};

struct Position
{
    int row;
    int col;
};

class Maze
{
public:

    Maze()
    {
        // Add boundary walls

        for (int r = 0; r < MAZE_SIZE; r++)
        {
            grid[r][0].walls[(int)Wall::WEST] = true;
            grid[r][MAZE_SIZE - 1].walls[(int)Wall::EAST] = true;
        }

        for (int c = 0; c < MAZE_SIZE; c++)
        {
            grid[0][c].walls[(int)Wall::NORTH] = true;
            grid[MAZE_SIZE - 1][c].walls[(int)Wall::SOUTH] = true;
        }
    }

    constexpr int toIndex(Wall wall) const
    {
        return static_cast<int>(wall);
    }

    Wall oppositeWall(Wall wall) const
    {
        switch(wall)
        {
            case Wall::NORTH:
                return Wall::SOUTH;

            case Wall::SOUTH:
                return Wall::NORTH;

            case Wall::EAST:
                return Wall::WEST;

            case Wall::WEST:
                return Wall::EAST;
        }
        return Wall::NORTH; // safety
    }

    bool isInside(int row, int col) const
    {
        return row >= 0 &&
               row < MAZE_SIZE &&
               col >= 0 &&
               col < MAZE_SIZE;
    }

    bool hasWall(int row, int col, Wall wall) const
    {
        return grid[row][col].walls[toIndex(wall)];
    }

    Position getNeighbourPosition(int row, int col, Wall wall) const
    {
        switch(wall)
        {
            case Wall::NORTH:
                return {row - 1, col};

            case Wall::EAST:
                return {row, col + 1};

            case Wall::SOUTH:
                return {row + 1, col};

            case Wall::WEST:
                return {row, col - 1};
        }
        return {row,col};
    }

    void setWall(int row, int col, Wall wall, bool exists)
    {
        // Set wall for current cell
        grid[row][col].walls[toIndex(wall)] = exists;


        // Find neighbouring cell
        Position neighbour = getNeighbourPosition(row,col,wall);


        // Update opposite wall if neighbour exists
        if(isInside(neighbour.row, neighbour.col))
        {
            grid[neighbour.row][neighbour.col]
            .walls[toIndex(oppositeWall(wall))] = exists;
        }
    }

    int getNeighbours(Position current, Position neighbours[]) const
    {
        int count = 0;
        int row = current.row;
        int col = current.col;

        // North
        if (!hasWall(row, col, Wall::NORTH) &&
            isInside(row - 1, col))
        {
            neighbours[count++] = {row - 1, col};
        }
        // East
        if (!hasWall(row, col, Wall::EAST) &&
            isInside(row, col + 1))
        {
            neighbours[count++] = {row, col + 1};
        }
        // South
        if (!hasWall(row, col, Wall::SOUTH) &&
            isInside(row + 1, col))
        {
            neighbours[count++] = {row + 1, col};
        }
        // West
        if (!hasWall(row, col, Wall::WEST) &&
            isInside(row, col - 1))
        {
            neighbours[count++] = {row, col - 1};
        }
        return count;
    }

    void printMaze() const
    {
        // Print top walls of every row
        for(int r = 0; r < MAZE_SIZE; r++)
        {
            // Print north walls
            for(int c = 0; c < MAZE_SIZE; c++)
            {
                Serial.print("+");

                if(hasWall(r,c,Wall::NORTH))
                    Serial.print("---");
                else
                    Serial.print("   ");
            }

            Serial.println("+");


            // Print west/east walls and cells
            for(int c = 0; c < MAZE_SIZE; c++)
            {
                if(hasWall(r,c,Wall::WEST))
                    Serial.print("|");
                else
                    Serial.print(" ");

                Serial.print("   ");
            }


            // Right boundary
            if(hasWall(r,MAZE_SIZE-1,Wall::EAST))
                Serial.println("|");
            else
                Serial.println(" ");
        }


        // Print bottom boundary
        for(int c = 0; c < MAZE_SIZE; c++)
        {
            Serial.print("+");

            if(hasWall(MAZE_SIZE-1,c,Wall::SOUTH))
                Serial.print("---");
            else
                Serial.print("   ");
        }

        Serial.println("+");
    }

private:

    Cell grid[MAZE_SIZE][MAZE_SIZE];
};

}