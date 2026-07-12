#pragma once
#include <vector>

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

    constexpr int toIndex(Wall wall)
    {
        return static_cast<int>(wall);
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

    void setWall(int row, int col, Wall wall, bool exists)
    {
        grid[row][col].walls[toIndex(wall)] = exists;
    }

    std::vector<Cell> getNeighbours() const
    {

    }

private:

    Cell grid[MAZE_SIZE][MAZE_SIZE];
};

}