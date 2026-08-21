#pragma once

#include "Robot.hpp"
#include "Movement.hpp"
#include "Misc.hpp"


namespace mm {

// MAZE_N_ROWS may be different to MAZE_N_COLS when testing
static constexpr uint8_t MAZE_N_ROWS = 9;
static constexpr uint8_t MAZE_N_COLS = 9;
static constexpr uint8_t NUM_CELLS = MAZE_N_ROWS * MAZE_N_COLS - 12;

// Coordinates are (row, column), 0-indexed. Row increases southward,
// column increases eastward. North points from (1, 0) to (0, 0).
struct Coord {
    uint8_t row : 4;
    uint8_t col : 4;
    // uint8_t row, col;
};
static_assert(sizeof(Coord) == sizeof(uint8_t));

struct Cell {
    uint8_t valid_cell : 1;
    uint8_t walls : 4;
    uint8_t visited : 1;
};
static_assert(sizeof(Cell) == sizeof(uint8_t));

enum Direction : uint8_t {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

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
            for (const auto& [row, col] : invalid_cells) {
                if (row == i and col == j) {
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

    maze.start_coord = {1, 3};
    maze.start_heading = SOUTH;

    maze.goal_coord = {1, 4};

    maze.visited_count = 0;

    robot_heading = maze.start_heading;
}

// Screen is 128x64px == 16x8 tiles (8x8px each). One tile per maze column
// (MAZE_N_COLS <= 16); one tile per maze row, but MAZE_N_ROWS may exceed the
// 8 available tile rows, so the window scrolls to keep the robot's current
// row in view. The remaining tile columns to the right are used for the %
// readout.
static constexpr uint8_t MAZE_VIS_TILE_ROWS = 8;
static constexpr uint8_t MAZE_VIS_TILE_COLS = MAZE_N_COLS;

// Draws a coarse view of `maze` showing the walls mapped so far (not
// visited-vs-unvisited fill), plus a marker on the robot's current cell,
// over a window of MAZE_VIS_TILE_ROWS rows scrolled to follow
// robot_pos.row, plus a % mapped readout derived from maze.visited_count /
// NUM_CELLS. Each cell is an 8x8 tile; wall bits (0=N,1=E,2=S,3=W) are drawn
// as the corresponding edge of the tile (top/bottom/left/right), so
// adjacent cells' shared walls line up into a real maze outline. Screen row
// 0 (top) is maze row `window_start`, i.e. (0, 0) is top-left and column
// increases rightward, matching the (row, col) convention above.
void draw_maze_visualisation(void) {
    auto& robot = GET_ROBOT();

    int16_t max_start = (int16_t)MAZE_N_ROWS - (int16_t)MAZE_VIS_TILE_ROWS;
    if (max_start < 0) max_start = 0;
    int16_t window_start = (int16_t)robot_pos.row - (int16_t)(MAZE_VIS_TILE_ROWS / 2);
    if (window_start < 0) window_start = 0;
    if (window_start > max_start) window_start = max_start;

    robot.oled.clear();

    uint8_t row_buf[MAZE_VIS_TILE_COLS * 8];
    for (uint8_t ty = 0; ty < MAZE_VIS_TILE_ROWS; ty++) {
        uint8_t row = (uint8_t)window_start + ty;

        for (uint8_t col = 0; col < MAZE_VIS_TILE_COLS; col++) {
            uint8_t tile[8] = {0, 0, 0, 0, 0, 0, 0, 0};

            if (row < MAZE_N_ROWS) {
                const Cell& cell = maze.cells[row][col];

                if (cell.valid_cell) {
                    // Byte index = pixel column (x), bit index = pixel row (y).
                    if (cell.walls & (1 << 0)) for (uint8_t x = 0; x < 8; x++) tile[x] |= 0x01; // N: top edge
                    if (cell.walls & (1 << 2)) for (uint8_t x = 0; x < 8; x++) tile[x] |= 0x80; // S: bottom edge
                    if (cell.walls & (1 << 3)) tile[0] = 0xFF; // W: left edge
                    if (cell.walls & (1 << 1)) tile[7] = 0xFF; // E: right edge

                    bool is_robot = (row == robot_pos.row and col == robot_pos.col);
                    if (is_robot) {
                        // Filled block in the middle marks the robot's current cell.
                        tile[3] |= 0x3C;
                        tile[4] |= 0x3C;
                    }
                }
            }

            memcpy(&row_buf[col * 8], tile, 8);
        }

        robot.oled.drawTile(0, ty, MAZE_VIS_TILE_COLS, row_buf);
    }

    // Large digits are 2x3 tile cells each; 3 chars (up to "100") exactly
    // fills the 6 tile columns left of the maze (text_col..15), so no '%'
    // sign is appended there - it'd push a 4th char off the right edge of
    // the screen. A small '%' is printed underneath instead.
    uint8_t percent = (uint8_t)((uint16_t)maze.visited_count * 100 / NUM_CELLS);
    uint8_t text_col = MAZE_VIS_TILE_COLS + 1;
    robot.oled.printLarge(text_col, 2, "%3d", percent);
    robot.oled.print(text_col + 2, 5, "%%");
}

bool drive_to_neighbour(int8_t chosen_dir) {
    int8_t delta = (chosen_dir - robot_heading + 4) % 4;
    float rotation;

    switch (delta) {
        case 1:
            rotation = -90.0f;
            break;

        case 2:
            rotation = 180.0f;
            break;

        case 3:
            rotation = 90.0f;
            break;

        default:
            rotation = 0.0f;
            break;
    }

    if (rotation != 0.0f) {
        if (fabs(rotation) >= 135.0f) {
            robot_rotate(rotation, 2200, 90);
        } else {
            robot_rotate(rotation, 1100, 90);
        }
    }

    // Double check if wall in front. Guard with frontHasReading(): the
    // sentinel -1 (no reading landed yet) otherwise always satisfies
    // "<= 90" and reads as a false wall right after boot.
    auto& robot = GET_ROBOT();
    robot.lidar_system.update();
    bool wall_to_front = robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 100;
    if (wall_to_front) {
        robot_rotate(-rotation, 2200, 90);
        return false;
    }

    robot_heading = chosen_dir;
    robot_drive_straight_with_lidars_no_profile_soft_start(CELL_SIZE_MM, 5000, 100, true);

    return true;
}


void maze_dfs(void) {
    auto& robot = GET_ROBOT();

    Coord current = maze.start_coord;
    robot_pos = current;
    maze.cells[current.row][current.col].visited = 1;
    maze.visited_count++;

    while (true) {
        draw_maze_visualisation();

        static Stack<Coord, NUM_CELLS> dfs_stack;

        robot.gyroscope.update();
        robot.lidar_system.update();

        // Guard with HasReading(): the -1 "no reading yet" sentinel
        // otherwise always satisfies "< 90" and reads as a false wall on
        // every side right after boot, before any sensor has a real sample.
        bool wall_to_front = robot.lidar_system.frontHasReading() && robot.lidar_system.readFront() <= 100;
        bool wall_to_left = robot.lidar_system.frontHasReading() && robot.lidar_system.readLeft() <= 100;
        bool wall_to_right = robot.lidar_system.frontHasReading() && robot.lidar_system.readRight() <= 100;

        if (wall_to_front) maze.cells[current.row][current.col].walls |= (1 << robot_heading);
        if (wall_to_left) maze.cells[current.row][current.col].walls |= (1 << ((robot_heading + 3) % 4));
        if (wall_to_right) maze.cells[current.row][current.col].walls |= (1 << ((robot_heading + 1) % 4));

        // Direction index: 0=N, 1=E, 2=S, 3=W. North decreases row, East increases column.
        int8_t d_row[4] = {-1, 0, 1, 0};
        int8_t d_col[4] = {0, 1, 0, -1};

        int8_t chosen_dir = -1;

        // Search left, right, forward, back (relative to robot_heading)
        // rather than fixed compass order - matches drive_to_neighbour's
        // delta encoding, where (heading+3)%4=left, (heading+1)%4=right,
        // heading=forward, (heading+2)%4=back.
        uint8_t dir_priority[4] = {
            static_cast<uint8_t>((robot_heading + 3) % 4),
            static_cast<uint8_t>((robot_heading + 1) % 4),
            robot_heading,
            static_cast<uint8_t>((robot_heading + 2) % 4)
        };

        // Find unvisited neighbour
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t dir = dir_priority[i];
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
                robot_pos = current;

                maze.cells[current.row][current.col].visited = 1;
                maze.visited_count++;
            } else {
                maze.cells[current.row][current.col].walls |= (1 << chosen_dir);
                (void)dfs_stack.pop();
            }
        } else {
            if (dfs_stack.isEmpty()) {
                robot.oled.clear();
                robot.oled.print(0, 0, "Done mapping.\n");
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
            robot_pos = current;
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

    static RingBuffer<uint8_t, NUM_CELLS> bfs_queue;
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
    auto& robot = GET_ROBOT();
    robot.oled.clear();
    robot.oled.printLarge(0, 0, "Task 4.3");
    delayWhileUpdating(1000UL);

    maze_setup();
    maze_dfs();
    shortest_path(maze.start_coord, maze.goal_coord);
}

}

