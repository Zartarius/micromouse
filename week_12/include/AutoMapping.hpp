#pragma once

#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"

// MAZE_N_ROWS may be different to MAZE_N_COLS when testing
#define MAZE_N_ROWS 9
#define MAZE_N_COLS 9
#define CELL_SIZE_MM 180.0f

namespace mm {

// Coordinates are (row, column), 0-indexed. Row increases southward,
// column increases eastward. North points from (1, 0) to (0, 0).
struct Coord {
    uint8_t row, col;
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
    uint8_t start_heading; // 0=N, 1=E, 2=S, 3=W. North decreases row, East increases column.
};

Maze maze;
static_assert(sizeof(maze.cells) == MAZE_N_ROWS * MAZE_N_COLS * sizeof(maze.cells[0][0]));

// Robot position
Coord robot_pos;
uint8_t robot_heading;

void maze_setup(void) {
    memset(maze.cells, 0, sizeof(maze.cells));

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
                if (cell.row == i and cell.col == j) {
                    valid_cell = false;
                    break;
                }
            }
            maze.cells[i][j].valid_cell = valid_cell;
        }
    }

    // Direction index: 0=N, 1=E, 2=S, 3=W. North decreases row, East increases column.
    int8_t d_row[4] = {-1, 0, 1, 0};
    int8_t d_col[4] = {0, 1, 0, -1};

    for (int i = 0; i < MAZE_N_ROWS; i++) {
        for (int j = 0; j < MAZE_N_COLS; j++) {
            for (uint8_t dir = 0; dir < 4; dir++) {
                int8_t ni = i + d_row[dir];
                int8_t nj = j + d_col[dir];

                bool out_of_bounds = (ni < 0 or ni >= MAZE_N_ROWS or nj < 0 or nj >= MAZE_N_COLS);
                bool neighbour_invalid = !out_of_bounds && !maze.cells[ni][nj].valid_cell;

                if (out_of_bounds or neighbour_invalid) {
                    maze.cells[i][j].walls |= (1 << dir);
                }
            }
        }
    }

    maze.start_coord = {1, 1};
    maze.start_heading = 2;

    maze.goal_coord = {7, 7};

    maze.visited_count = 0;

    robot_heading = maze.start_heading;
}

bool drive_to_neighbour(int8_t chosen_dir) {
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
        robot_rotate(rotation, 1700, 100);
    }

    // Double check if wall in front
    auto& robot = GET_ROBOT();
    robot.lidar_system.update();
    bool wall_to_front = robot.lidar_system.readFront() <= 90;
    if (wall_to_front) {
        robot_rotate(-rotation, 1700, 100);
        return false;
    }

    robot_heading = chosen_dir;
    // robot_drive_straight(CELL_SIZE_MM, 5000);
    robot_drive_straight_with_lidars_no_profile(CELL_SIZE_MM, 5000, 100, true);

    return true;
}


void maze_dfs(void) {
    auto& robot = GET_ROBOT();

    Coord current = maze.start_coord;
    maze.cells[current.row][current.col].visited = 1;
    maze.visited_count++;

    while (true) {
        static Stack<Coord, MAZE_N_ROWS * MAZE_N_COLS> dfs_stack;

        restart:

        robot.gyroscope.update();
        robot.lidar_system.update();

        bool wall_to_front = robot.lidar_system.readFront() < 90;
        bool wall_to_left = robot.lidar_system.readLeft() < 90;
        bool wall_to_right = robot.lidar_system.readRight() < 90;

        if (wall_to_front) maze.cells[current.row][current.col].walls |= (1 << robot_heading);
        if (wall_to_left) maze.cells[current.row][current.col].walls |= (1 << ((robot_heading + 3) % 4));
        if (wall_to_right) maze.cells[current.row][current.col].walls |= (1 << ((robot_heading + 1) % 4));

        // Direction index: 0=N, 1=E, 2=S, 3=W. North decreases row, East increases column.
        int8_t d_row[4] = {-1, 0, 1, 0};
        int8_t d_col[4] = {0, 1, 0, -1};

        int8_t chosen_dir = -1;

        // Find unvisited neighbour
        for (uint8_t dir = 0; dir < 4; dir++) {
            bool wall_present = maze.cells[current.row][current.col].walls & (1 << dir);
            if (wall_present) continue;

            int8_t n_row = current.row + d_row[dir];
            int8_t n_col = current.col + d_col[dir];

            // once we add virtual walls this check should become redundant
            if (n_row < 0 or n_row >= MAZE_N_ROWS or n_col < 0 or n_col >= MAZE_N_COLS) continue;

            if (maze.cells[n_row][n_col].visited) continue;

            chosen_dir = dir;
            break;
        }

        if (chosen_dir != -1) {
            dfs_stack.push(current);

            if (drive_to_neighbour(chosen_dir)) {
                current.row += d_row[chosen_dir];
                current.col += d_col[chosen_dir];

                maze.cells[current.row][current.col].visited = 1;
                maze.visited_count++;
            } else {
                maze.cells[current.row][current.col].walls |= (1 << chosen_dir);
                (void)dfs_stack.pop();
                goto restart;
            }
        } else {
            if (dfs_stack.top == 0) {
                robot.oled.clear();
                robot.oled.print(0, 0, "Done mapping.");
                break;
            }

            Coord back = dfs_stack.pop();

            int8_t back_dir = -1;
            for (uint8_t dir = 0; dir < 4; dir++) {
                if (current.row + d_row[dir] == back.row and current.col + d_col[dir] == back.col) {
                    back_dir = dir;
                    break;
                }
            }

            // This path was already driven once (forward), so it should be clear.
            // Retry instead of recording a false wall or desyncing `current` from
            // the robot's real position on a failed drive.
            bool reached_back = false;
            for (uint8_t attempt = 0; attempt < 5 and !reached_back; attempt++) {
                reached_back = drive_to_neighbour(back_dir);
            }

            if (!reached_back) {
                robot.oled.clear();
                robot.oled.print(0, 0, "Backtrack blocked.");
                break;
            }

            current = back;
        }
    }
}


void shortest_path(const Coord& start, const Coord& end) {
    auto& robot = GET_ROBOT();

    int8_t d_row[4] = {-1, 0, 1, 0};
    int8_t d_col[4] = {0, 1, 0, -1};

    static int8_t dir_toward_end[MAZE_N_ROWS][MAZE_N_COLS];
    for (int i = 0; i < MAZE_N_ROWS; i++) {
        for (int j = 0; j < MAZE_N_COLS; j++) {
            dir_toward_end[i][j] = -1; // -1 == not yet reached by the BFS
        }
    }

    static RingBuffer<uint8_t, MAZE_N_ROWS * MAZE_N_COLS> bfs_queue;
    bfs_queue.clear();
    bfs_queue.push(end.row * MAZE_N_COLS + end.col);
    dir_toward_end[end.row][end.col] = -2; // reached, but no onward direction needed

    while (!bfs_queue.isEmpty()) {
        uint8_t idx;
        bfs_queue.pop(idx);
        uint8_t row = idx / MAZE_N_COLS;
        uint8_t col = idx % MAZE_N_COLS;

        for (uint8_t dir = 0; dir < 4; dir++) {
            // A neighbour N reaches (row, col) by driving `dir`
            int8_t n_row = row - d_row[dir];
            int8_t n_col = col - d_col[dir];

            if (n_row < 0 or n_row >= MAZE_N_ROWS or n_col < 0 or n_col >= MAZE_N_COLS) continue;
            if (dir_toward_end[n_row][n_col] != -1) continue;

            bool wall_present = maze.cells[n_row][n_col].walls & (1 << dir);
            if (wall_present) continue;

            dir_toward_end[n_row][n_col] = dir;
            bfs_queue.push(n_row * MAZE_N_COLS + n_col);
        }
    }

    bool start_is_end = (start.row == end.row and start.col == end.col);
    if (!start_is_end and dir_toward_end[start.row][start.col] == -1) {
        robot.oled.clear();
        robot.oled.print(0, 0, "No path found.");
        return;
    }

    Coord current = start;
    while (!(current.row == end.row and current.col == end.col)) {
        int8_t dir = dir_toward_end[current.row][current.col];

        bool reached = false;
        for (uint8_t attempt = 0; attempt < 5 and !reached; attempt++) {
            reached = drive_to_neighbour(dir);
        }

        if (!reached) {
            robot.oled.clear();
            robot.oled.print(0, 0, "Path blocked.");
            return;
        }

        current.row += d_row[dir];
        current.col += d_col[dir];
    }

    robot.oled.clear();
    robot.oled.print(0, 0, "Reached goal.");
}

void do_auto_mapping(void) {
    maze_setup();
    maze_dfs();
    shortest_path(maze.start_coord, maze.goal_coord);
}

}

