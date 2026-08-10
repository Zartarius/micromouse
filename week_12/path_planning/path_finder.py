from collections import deque


def bfs(graph, start, goal):
    """
    Find the shortest path between start and goal using BFS.

    Since every maze edge has equal cost, BFS produces a minimum-step
    path through the maze.
    """

    queue = deque([start])
    visited = {start}

    # Parent links allow the final path to be reconstructed.
    parent = {start: None}

    while queue:

        current = queue.popleft()

        if current == goal:
            break

        for neighbour in graph.get_neighbors(current):

            if neighbour not in visited:

                visited.add(neighbour)
                parent[neighbour] = current
                queue.append(neighbour)

    if goal not in parent:
        return None

    # Reconstruct path by following parent links backwards.
    path = []
    current = goal

    while current is not None:

        path.append(current)
        current = parent[current]

    path.reverse()

    return path


def path_to_commands(path, start_direction="N", cols=9):
    """
    Convert a sequence of maze cells into robot movement commands.

    The robot can turn left, turn right, or move forward.
    """

    directions = ["N", "E", "S", "W"]

    current_direction = start_direction
    commands = []

    for i in range(len(path) - 1):

        current_node = path[i]
        next_node = path[i + 1]

        current_row = current_node // cols
        current_col = current_node % cols

        next_row = next_node // cols
        next_col = next_node % cols

        movement = (
            next_row - current_row,
            next_col - current_col
        )

        if movement == (-1, 0):
            required_direction = "N"

        elif movement == (0, 1):
            required_direction = "E"

        elif movement == (1, 0):
            required_direction = "S"

        elif movement == (0, -1):
            required_direction = "W"

        else:
            raise ValueError(
                f"Invalid movement from {current_node} "
                f"to {next_node}"
            )

        current_index = directions.index(current_direction)
        required_index = directions.index(required_direction)

        turn = (required_index - current_index) % 4

        if turn == 1:
            commands.append("r")

        elif turn == 2:
            commands.extend(["r", "r"])

        elif turn == 3:
            commands.append("l")

        commands.append("f")

        current_direction = required_direction

    return commands