#pragma once

#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"

// MAZE_N_ROWS may be different to MAZE_N_COLS when testing
#define MAZE_N_ROWS 9
#define MAZE_N_COLS 9
#define CELL_SIZE_MM 180.0f

namespace mm {

struct Coord {
    uint8_t x, y;
};

struct Cell {
    uint8_t valid_cell : 1;
    uint8_t walls : 4;
    uint8_t visited : 1;
};

static_assert(sizeof(Cell) == sizeof(uint8_t));

struct Maze {
    Cell cells[MAZE_N_ROWS][MAZE_N_COLS];
    uint8_t visited_count;
    Coord goal_coord;
    Coord start_coord;
    uint8_t start_heading; // 0=N, 1=E, 2=S, 3=W. Assuming North to point towards the larger y values.
};

Maze maze;
Stack<Coord, MAZE_N_ROWS * MAZE_N_COLS> dfs_stack;
// Robot position
Coord robot_pos;
uint8_t robot_heading = 0;

void maze_setup(void) {
    for (int i = 0; i < MAZE_N_ROWS; i++) {
        for (int j = 0; j < MAZE_N_COLS; j++) {
            maze.cells[i][j] = {0, 0, 0};
        }
    }

    Coord invalid_cells[] = {
        {0, 0}, {0, 1}, {1, 0},
        {0, 8}, {0, 7}, {1, 8},
        {8, 8}, {8, 7}, {7, 8},
        {8, 0}, {7, 0}, {8, 1}
    };

    for (int i = 0; i < MAZE_N_ROWS; i++) {
        for (int j = 0; j < MAZE_N_COLS; j++) {
            bool valid_cell = true;
            for (Coord& cell : invalid_cells) {
                if (cell.x == i and cell.y == j) {
                    valid_cell = false;
                    break;
                }
            }
            maze.cells[i][j].valid_cell = valid_cell;
        }
    }


    int8_t dx[4] = {0, 1, 0, -1};
    int8_t dy[4] = {1, 0, -1, 0};

    for (int i = 0; i < MAZE_N_ROWS; i++) {
        for (int j = 0; j < MAZE_N_COLS; j++) {
            for (uint8_t dir = 0; dir < 4; dir++) {
                int8_t ni = i + dx[dir];
                int8_t nj = j + dy[dir];

                bool out_of_bounds = (ni < 0 or ni >= MAZE_N_ROWS or nj < 0 or nj >= MAZE_N_COLS);
                bool neighbour_invalid = !out_of_bounds && !maze.cells[ni][nj].valid_cell;

                if (out_of_bounds or neighbour_invalid) {
                    maze.cells[i][j].walls |= (1 << dir);
                }
            }
        }
    }


    maze.start_coord = {1, 1};
    maze.start_heading = 0;

    maze.goal_coord = {7, 7};

    maze.visited_count = 0;
}

void drive_to_neighbour(int8_t chosen_dir) {
    int8_t delta = (chosen_dir - robot_heading + 4) % 4;
    float rotation = 0.0f;

    if (delta == 1) {
        rotation = -90.0f;   // turn right
    } else if (delta == 2) {
        rotation = 180.0f;  // turn around
    } else if (delta == 3) {
        rotation = 90.0f;  // turn left
    }

    if (rotation != 0.0f) {
        robot_rotate(rotation, 800);
    }

    robot_heading = chosen_dir;
    robot_drive_straight_with_lidars_no_profile(CELL_SIZE_MM, 5000, 130, true);
}


void maze_dfs(void) {
    auto& robot = GET_ROBOT();

    Coord current = maze.start_coord;
    maze.cells[current.x][current.y].visited = 1;
    maze.visited_count++;

    while (true) {
        robot.gyroscope.update();
        robot.lidar_system.update();

        bool wall_to_front = robot.lidar_system.readFront() < 60;
        bool wall_to_left = robot.lidar_system.readLeft() < 60;
        bool wall_to_right = robot.lidar_system.readRight() < 60;

        if (wall_to_front) maze.cells[current.x][current.y].walls |= (1 << robot_heading);
        if (wall_to_left) maze.cells[current.x][current.y].walls |= (1 << ((robot_heading + 3) % 4));
        if (wall_to_right) maze.cells[current.x][current.y].walls |= (1 << ((robot_heading + 1) % 4));

        int8_t dx[4] = {0, 1, 0, -1};
        int8_t dy[4] = {1, 0, -1, 0};

        int8_t chosen_dir = -1;

        // Find unvisited neighbour
        for (uint8_t dir = 0; dir < 4; dir++) {
            bool wall_present = maze.cells[current.x][current.y].walls & (1 << dir);
            if (wall_present) continue;

            int8_t nx = current.x + dx[dir];
            int8_t ny = current.y + dy[dir];

            // once we add virtual walls this check should become redundant
            if (nx < 0 or nx >= MAZE_N_ROWS or ny < 0 or ny >= MAZE_N_COLS) continue;

            if (maze.cells[nx][ny].visited) continue;

            chosen_dir = dir;
            break;
        }

        if (chosen_dir != -1) {
            dfs_stack.push(current);

            drive_to_neighbour(chosen_dir);

            current.x += dx[chosen_dir];
            current.y += dy[chosen_dir];

            maze.cells[current.x][current.y].visited = 1;
            maze.visited_count++;
        } else {
            if (dfs_stack.top == 0) {
                robot.oled.clear();
                robot.oled.print(0, 0, "Done mapping.");
                break;
            }

            Coord back = dfs_stack.pop();

            int8_t back_dir = -1;
            for (uint8_t dir = 0; dir < 4; dir++) {
                if (current.x + dx[dir] == back.x and current.y + dy[dir] == back.y) {
                    back_dir = dir;
                    break;
                }
            }

            drive_to_neighbour(back_dir);
            current = back;
        }
    }
}

void do_auto_mapping(void) {
    maze_setup();
    maze_dfs();
}

}

