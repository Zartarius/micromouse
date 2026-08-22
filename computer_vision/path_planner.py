"""
Maze graph representation and command-optimal path search.

`Graph`/`Node` hold the 9x9 maze as detected by `maze_vision.py` (one
node per cell, with N/S/E/W wall flags). The planner functions below
search that graph for the path that requires the fewest robot
commands (f = forward, r = turn right, l = turn left) and convert it
into the move sequence the firmware chains together.
"""

import heapq


class Node:

    def __init__(self, node_id, x, y):
        self.id = node_id
        self.x = x
        self.y = y

        self.north = False
        self.south = False
        self.east = False
        self.west = False

    def get_point(self):
        return self.x, self.y

    def get_ID(self):
        return self.id

    def get_walls(self):
        return {
            "N": self.north,
            "S": self.south,
            "E": self.east,
            "W": self.west
        }


class Graph:

    def __init__(self):
        self.nodes = {}
        self.edges = {}

    def add_node(self, node_id, x, y):
        self.nodes[node_id] = Node(node_id, x, y)
        self.edges.setdefault(node_id, {})

    def add_edge(self, node_id1, node_id2, weight):
        self.edges[node_id1][node_id2] = weight
        self.edges[node_id2][node_id1] = weight

    def remove_edge(self, node_id1, node_id2):
        self.edges.get(node_id1, {}).pop(node_id2, None)
        self.edges.get(node_id2, {}).pop(node_id1, None)

    def get_nodes(self):
        return self.nodes

    def get_edge_weight(self, node_id1, node_id2):
        return self.edges.get(node_id1, {}).get(node_id2, None)

    def get_neighbors(self, node_id):
        return self.edges.get(node_id, {})


def create_grid_graph(grid_size):
    """
    Create one graph node for each maze cell.
    """

    graph = Graph()

    for row in range(grid_size):
        for col in range(grid_size):
            graph.add_node(row * grid_size + col, col, row)

    return graph


def add_valid_edges(graph, grid_size):
    """
    Connect adjacent cells that do not have a wall between them.
    """

    for row in range(grid_size):
        for col in range(grid_size):
            node_id = row * grid_size + col
            node = graph.nodes[node_id]

            if col < grid_size - 1:
                east_node_id = row * grid_size + col + 1

                if not node.east:
                    graph.add_edge(node_id, east_node_id, 1)

            if row < grid_size - 1:
                south_node_id = (row + 1) * grid_size + col

                if not node.south:
                    graph.add_edge(node_id, south_node_id, 1)


def print_maze(graph, rows=9, cols=9):
    """
    Print an ASCII representation of the detected maze.
    """

    print("+" + "---+" * cols)

    for row in range(rows):
        line = ""

        for col in range(cols):
            node = graph.nodes[row * cols + col]
            line += "|" if node.west else " "
            line += "   "

        node = graph.nodes[row * cols + cols - 1]
        line += "|" if node.east else " "

        print(line)

        line = ""

        for col in range(cols):
            node = graph.nodes[row * cols + col]
            line += "+"
            line += "---" if node.south else "   "

        line += "+"

        print(line)


def print_maze_with_path(graph, path, rows=9, cols=9):
    """
    Print the digital maze with S, G and the calculated path.
    """

    path_set = set(path)
    start = path[0]
    goal = path[-1]

    print("+" + "---+" * cols)

    for row in range(rows):
        line = ""

        for col in range(cols):
            node_id = row * cols + col
            node = graph.nodes[node_id]

            line += "|" if node.west else " "

            if node_id == start:
                line += " S "
            elif node_id == goal:
                line += " G "
            elif node_id in path_set:
                line += " • "
            else:
                line += "   "

        node = graph.nodes[row * cols + (cols - 1)]
        line += "|" if node.east else " "

        print(line)

        line = ""

        for col in range(cols):
            node = graph.nodes[row * cols + col]
            line += "+"
            line += "---" if node.south else "   "

        line += "+"
        print(line)


_DIRECTIONS = ["N", "E", "S", "W"]


def _required_direction(current_node, neighbour, cols):
    current_row, current_col = current_node // cols, current_node % cols
    next_row, next_col = neighbour // cols, neighbour % cols

    movement = (next_row - current_row, next_col - current_col)

    if movement == (-1, 0):
        return "N"
    elif movement == (0, 1):
        return "E"
    elif movement == (1, 0):
        return "S"
    elif movement == (0, -1):
        return "W"
    else:
        raise ValueError(f"Invalid movement from {current_node} to {neighbour}")


def _turn_commands(current_direction, required_direction):
    current_index = _DIRECTIONS.index(current_direction)
    required_index = _DIRECTIONS.index(required_direction)

    turn = (required_index - current_index) % 4

    if turn == 0:
        return [], turn
    elif turn == 1:
        return ["r"], turn
    elif turn == 2:
        return ["r", "r"], turn
    else:
        return ["l"], turn


def command_optimal_path(graph, start, goal, start_direction="N", cols=9):
    """
    Find the path from start to goal that requires the minimum
    number of robot commands.

    The robot has three commands:
        f = move forward one cell
        r = turn right
        l = turn left

    The search state is:
        (node, current_direction)

    Therefore, arriving at the same node from different directions
    is treated as a different state.

    Returns:
        path, commands
    """

    start_state = (start, start_direction)

    queue = [(0, start, start_direction)]

    cost = {start_state: 0}
    parent = {start_state: None}
    parent_commands = {start_state: []}

    goal_state = None

    # =========================================================
    # DIJKSTRA
    # =========================================================

    while queue:
        current_cost, current_node, current_direction = heapq.heappop(queue)
        current_state = (current_node, current_direction)

        # Ignore outdated entries in the priority queue
        if current_cost != cost[current_state]:
            continue

        if current_node == goal:
            goal_state = current_state
            break

        for neighbour in graph.get_neighbors(current_node):
            required_direction = _required_direction(current_node, neighbour, cols)
            turn_commands, _ = _turn_commands(current_direction, required_direction)

            movement_commands = turn_commands + ["f"]
            new_cost = current_cost + len(movement_commands)
            new_state = (neighbour, required_direction)

            if new_state not in cost or new_cost < cost[new_state]:
                cost[new_state] = new_cost
                parent[new_state] = current_state
                parent_commands[new_state] = movement_commands

                heapq.heappush(queue, (new_cost, neighbour, required_direction))

    if goal_state is None:
        return None, None

    # =========================================================
    # RECONSTRUCT PATH + COMMANDS
    # =========================================================

    states = []
    current_state = goal_state

    while current_state is not None:
        states.append(current_state)
        current_state = parent[current_state]

    states.reverse()

    path = [state[0] for state in states]

    commands = []
    for state in states[1:]:
        commands.extend(parent_commands[state])

    return path, commands


def command_optimal_path_no_long_straights(
    graph, start, goal, start_direction="N", cols=9, max_open_streak=2
):
    """
    Same as command_optimal_path, but disallows any path that drives
    straight (no turn) through 3 or more consecutive cells which have
    no wall on either side (left/right relative to travel direction).

    Search state is now (node, direction, streak), where `streak` is
    the number of consecutive "open-sided" cells just travelled
    through in a straight line. Any transition that would push the
    streak to max_open_streak + 1 is pruned from the search.
    """

    def side_walls(node_obj, direction):
        """Return (left_wall, right_wall) relative to direction of travel."""
        walls = node_obj.get_walls()
        if direction == "N":
            return walls["W"], walls["E"]
        elif direction == "S":
            return walls["E"], walls["W"]
        elif direction == "E":
            return walls["N"], walls["S"]
        else:  # "W"
            return walls["S"], walls["N"]

    def is_open_cell(node_obj, direction):
        left, right = side_walls(node_obj, direction)
        return (not left) and (not right)

    start_state = (start, start_direction, 0)

    queue = [(0, start, start_direction, 0)]

    cost = {start_state: 0}
    parent = {start_state: None}
    parent_commands = {start_state: []}

    goal_state = None

    # =========================================================
    # DIJKSTRA
    # =========================================================

    while queue:
        current_cost, current_node, current_direction, current_streak = heapq.heappop(queue)
        current_state = (current_node, current_direction, current_streak)

        if current_cost != cost[current_state]:
            continue

        if current_node == goal:
            goal_state = current_state
            break

        for neighbour in graph.get_neighbors(current_node):
            required_direction = _required_direction(current_node, neighbour, cols)
            turn_commands, turn = _turn_commands(current_direction, required_direction)

            movement_commands = turn_commands + ["f"]
            new_cost = current_cost + len(movement_commands)

            # Update the "open straight-line" streak.
            neighbour_open = is_open_cell(graph.nodes[neighbour], required_direction)

            if turn == 0:
                new_streak = current_streak + 1 if neighbour_open else 0
            else:
                new_streak = 1 if neighbour_open else 0

            # Forbid 3+ consecutive open cells in a straight line.
            if new_streak > max_open_streak:
                continue

            new_state = (neighbour, required_direction, new_streak)

            if new_state not in cost or new_cost < cost[new_state]:
                cost[new_state] = new_cost
                parent[new_state] = current_state
                parent_commands[new_state] = movement_commands

                heapq.heappush(queue, (new_cost, neighbour, required_direction, new_streak))

    if goal_state is None:
        return None, None

    states = []
    current_state = goal_state

    while current_state is not None:
        states.append(current_state)
        current_state = parent[current_state]

    states.reverse()

    path = [state[0] for state in states]

    commands = []
    for state in states[1:]:
        commands.extend(parent_commands[state])

    return path, commands


def path_to_commands(path, start_direction="N", cols=9):
    """
    Convert a path (as returned by the command-optimal planners above)
    into robot commands.
    """

    if path is None:
        return []

    current_direction = start_direction
    commands = []

    for i in range(len(path) - 1):
        current_node, next_node = path[i], path[i + 1]

        required_direction = _required_direction(current_node, next_node, cols)
        turn_commands, _ = _turn_commands(current_direction, required_direction)

        commands.extend(turn_commands)
        commands.append("f")

        current_direction = required_direction

    return commands


def find_obst_course_nodes(top_left_node, width, cols=9):
    top_left_row = top_left_node // cols
    top_left_col = top_left_node % cols

    obstacle_nodes = set()

    for row in range(width):
        for col in range(width):
            obstacle_nodes.add((top_left_row + row) * cols + (top_left_col + col))

    return obstacle_nodes


def find_obstacle_course_entrances(graph, obstacle_nodes):

    entrances = []

    for node in obstacle_nodes:
        for neighbour in graph.get_neighbors(node):
            if neighbour not in obstacle_nodes:
                # (obstacle course node, maze node)
                entrances.append((node, neighbour))

    return entrances


def command_optimal_path_to_obstacles(graph, start, course_top_left, course_width, start_direction="N", cols=9):
    """
    Find the command-optimal path from start to the nearest entrance
    of the obstacle course (the `course_width` x `course_width` block
    of cells whose top-left cell is `course_top_left`).

    Returns:
        path, commands
    """

    obstacle_nodes = find_obst_course_nodes(course_top_left, course_width, cols)

    start_state = (start, start_direction)

    queue = [(0, start, start_direction)]

    cost = {start_state: 0}
    parent = {start_state: None}
    parent_commands = {start_state: []}

    goal_state = None

    # =========================================================
    # DIJKSTRA
    # =========================================================

    while queue:
        current_cost, current_node, current_direction = heapq.heappop(queue)
        current_state = (current_node, current_direction)

        # Ignore outdated entries in the priority queue
        if current_cost != cost[current_state]:
            continue

        for neighbour in graph.get_neighbors(current_node):
            required_direction = _required_direction(current_node, neighbour, cols)
            turn_commands, _ = _turn_commands(current_direction, required_direction)

            movement_commands = turn_commands + ["f"]
            new_cost = current_cost + len(movement_commands)
            new_state = (neighbour, required_direction)

            # Stop as soon as we step into the obstacle course.
            if current_node not in obstacle_nodes and neighbour in obstacle_nodes:
                goal_state = new_state
                parent[new_state] = current_state
                parent_commands[new_state] = movement_commands
                break

            if new_state not in cost or new_cost < cost[new_state]:
                cost[new_state] = new_cost
                parent[new_state] = current_state
                parent_commands[new_state] = movement_commands

                heapq.heappush(queue, (new_cost, neighbour, required_direction))

        if goal_state is not None:
            break

    if goal_state is None:
        return None, None

    states = []
    current_state = goal_state

    while current_state is not None:
        states.append(current_state)
        current_state = parent[current_state]

    states.reverse()

    path = [state[0] for state in states]

    commands = []
    for state in states[1:]:
        commands.extend(parent_commands[state])

    return path, commands


def convert_commands(commands):

    converted = []

    for command in commands:
        if command == "f":
            converted.append((180.00, 0.0))
        elif command == "r":
            converted.append((0.00, 90.0))
        elif command == "l":
            converted.append((0.00, -90.0))
        else:
            raise ValueError(f"Invalid command: {command}")

    return converted
