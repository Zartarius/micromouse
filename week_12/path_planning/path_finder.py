import heapq


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
        path
        commands
    """

    print("xyz")
    directions = ["N", "E", "S", "W"]

    # ---------------------------------------------------------
    # State = (node, direction)
    # ---------------------------------------------------------

    start_state = (start, start_direction)

    # Priority queue:
    # (cost, node, direction)

    queue = []

    heapq.heappush(
        queue,
        (0, start, start_direction)
    )

    # Minimum number of commands required to reach each state
    cost = {
        start_state: 0
    }

    # Parent state used to reconstruct the path
    parent = {
        start_state: None
    }

    # Commands used to reach each state
    parent_commands = {
        start_state: []
    }

    goal_state = None

    # =========================================================
    # DIJKSTRA
    # =========================================================

    while queue:

        current_cost, current_node, current_direction = (
            heapq.heappop(queue)
        )

        current_state = (
            current_node,
            current_direction
        )
        
        # Ignore outdated entries in the priority queue
        if current_cost != cost[current_state]:
            continue

        # -----------------------------------------------------
        # Goal reached
        # -----------------------------------------------------
        print("bb")
        if current_node == goal:
            goal_state = current_state
            break

        # -----------------------------------------------------
        # Explore neighbouring cells
        # -----------------------------------------------------

        for neighbour in graph.get_neighbors(current_node):
            current_row = current_node // cols
            current_col = current_node % cols

            next_row = neighbour // cols
            next_col = neighbour % cols

            movement = (
                next_row - current_row,
                next_col - current_col
            )

            # -------------------------------------------------
            # Determine direction required to reach neighbour
            # -------------------------------------------------

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
                    f"to {neighbour}"
                )

            current_index = directions.index(
                current_direction
            )

            required_index = directions.index(
                required_direction
            )

            # -------------------------------------------------
            # Determine required turns
            # -------------------------------------------------

            turn = (
                required_index - current_index
            ) % 4

            if turn == 0:

                turn_commands = []

            elif turn == 1:

                turn_commands = ["r"]

            elif turn == 2:

                turn_commands = ["r", "r"]

            else:

                turn_commands = ["l"]

            # -------------------------------------------------
            # Every movement requires one forward command
            # -------------------------------------------------

            movement_commands = (
                turn_commands + ["f"]
            )

            movement_cost = len(
                movement_commands
            )

            new_cost = (
                current_cost
                +
                movement_cost
            )

            new_state = (
                neighbour,
                required_direction
            )

            # -------------------------------------------------
            # Update if this is the best way to reach the state
            # -------------------------------------------------

            if (
                new_state not in cost
                or
                new_cost < cost[new_state]
            ):

                cost[new_state] = new_cost

                parent[new_state] = current_state

                parent_commands[new_state] = (
                    movement_commands
                )

                heapq.heappush(
                    queue,
                    (
                        new_cost,
                        neighbour,
                        required_direction
                    )
                )

    # =========================================================
    # NO PATH
    # =========================================================

    if goal_state is None:

        return None, None

    # =========================================================
    # RECONSTRUCT PATH
    # =========================================================

    states = []

    current_state = goal_state

    while current_state is not None:

        states.append(current_state)

        current_state = parent[current_state]

    states.reverse()

    # ---------------------------------------------------------
    # Extract node IDs
    # ---------------------------------------------------------

    path = [
        state[0]
        for state in states
    ]

    # =========================================================
    # RECONSTRUCT COMMANDS
    # =========================================================

    commands = []

    for state in states[1:]:

        commands.extend(
            parent_commands[state]
        )

    return path, commands


def path_to_commands(path, start_direction="N", cols=9):
    """
    Convert a path into robot commands.

    This function is kept with the same name as your original
    version so the rest of your code does not need to change.

    It now assumes that 'path' has already been selected by the
    command-optimal Dijkstra planner.
    """

    if path is None:

        return []

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

        current_index = directions.index(
            current_direction
        )

        required_index = directions.index(
            required_direction
        )

        turn = (
            required_index - current_index
        ) % 4

        if turn == 1:

            commands.append("r")

        elif turn == 2:

            commands.extend(["r", "r"])

        elif turn == 3:

            commands.append("l")

        commands.append("f")

        current_direction = required_direction

    return commands


def command_optimal_path_to_obstacles(graph, start, course_top_left, course_width, start_direction="N", cols=9):
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
        path
        commands
    """


    obstacle_nodes = find_obst_course_nodes(course_top_left, course_width, cols)
    ##this finds which nodes are in the obstacle course using course top left + course width coordinates given
    # to construct obstacle course
    
    directions = ["N", "E", "S", "W"]

    # ---------------------------------------------------------
    # State = (node, direction)
    # ---------------------------------------------------------

    start_state = (start, start_direction)

    # Priority queue:
    # (cost, node, direction)

    queue = []

    heapq.heappush(
        queue,
        (0, start, start_direction)
    )

    # Minimum number of commands required to reach each state
    cost = {
        start_state: 0
    }

    # Parent state used to reconstruct the path
    parent = {
        start_state: None
    }

    # Commands used to reach each state
    parent_commands = {
        start_state: []
    }

    goal_state = None

    # =========================================================
    # DIJKSTRA
    # =========================================================

    while queue:

        current_cost, current_node, current_direction = (
            heapq.heappop(queue)
        )

        current_state = (
            current_node,
            current_direction
        )

        # Ignore outdated entries in the priority queue
        if current_cost != cost[current_state]:
            continue

        # -----------------------------------------------------
        # Goal reached
        # -----------------------------------------------------

        # if current_node == goal:

        #     goal_state = current_state
        #     break

        # -----------------------------------------------------
        # Explore neighbouring cells
        # -----------------------------------------------------

        for neighbour in graph.get_neighbors(current_node):

            current_row = current_node // cols
            current_col = current_node % cols

            next_row = neighbour // cols
            next_col = neighbour % cols

            movement = (
                next_row - current_row,
                next_col - current_col
            )

            # -------------------------------------------------
            # Determine direction required to reach neighbour
            # -------------------------------------------------

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
                    f"to {neighbour}"
                )

            current_index = directions.index(
                current_direction
            )

            required_index = directions.index(
                required_direction
            )

            # -------------------------------------------------
            # Determine required turns
            # -------------------------------------------------

            turn = (
                required_index - current_index
            ) % 4

            if turn == 0:

                turn_commands = []

            elif turn == 1:

                turn_commands = ["r"]

            elif turn == 2:

                turn_commands = ["r", "r"]

            else:

                turn_commands = ["l"]

            # -------------------------------------------------
            # Every movement requires one forward command
            # -------------------------------------------------

            movement_commands = (
                turn_commands + ["f"]
            )

            movement_cost = len(
                movement_commands
            )

            new_cost = (
                current_cost
                +
                movement_cost
            )

            new_state = (
                neighbour,
                required_direction
            )

            ## check for obstacle course entrance
            if (current_node not in obstacle_nodes and neighbour in obstacle_nodes):
                goal_state = new_state
                parent[new_state] = current_state
                parent_commands[new_state] = (movement_commands)
                break

            # -------------------------------------------------
            # Update if this is the best way to reach the state
            # -------------------------------------------------

            if (
                new_state not in cost
                or
                new_cost < cost[new_state]
            ):

                cost[new_state] = new_cost

                parent[new_state] = current_state

                parent_commands[new_state] = (
                    movement_commands
                )

                heapq.heappush(
                    queue,
                    (
                        new_cost,
                        neighbour,
                        required_direction
                    )
                )
        #shld stop once entrance has been found
        if goal_state is not None:
            break

    # =========================================================
    # NO PATH
    # =========================================================

    if goal_state is None:

        return None, None

    # =========================================================
    # RECONSTRUCT PATH
    # =========================================================

    states = []

    current_state = goal_state

    while current_state is not None:

        states.append(current_state)

        current_state = parent[current_state]

    states.reverse()

    # ---------------------------------------------------------
    # Extract node IDs
    # ---------------------------------------------------------

    path = [
        state[0]
        for state in states
    ]

    # =========================================================
    # RECONSTRUCT COMMANDS
    # =========================================================

    commands = []

    for state in states[1:]:

        commands.extend(
            parent_commands[state]
        )

    return path, commands

def find_obst_course_nodes(top_left_node, width, cols=9):
    top_left_row = top_left_node // cols
    top_left_col = top_left_node % cols

    obstacle_nodes = set()

    for row in range(width):
        for col in range(width):

            node = (
                (top_left_row + row) * cols
                +
                (top_left_col + col)
            )

            obstacle_nodes.add(node)

    return obstacle_nodes

def find_obstacle_course_entrances(graph, obstacle_nodes):

    entrances = []

    for node in obstacle_nodes:

        for neighbour in graph.get_neighbors(node):

            if neighbour not in obstacle_nodes:

                entrances.append(
                    (node, neighbour)
                    #this is (obstacle course node, maze node)
                )

    return entrances

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
            raise ValueError(
                f"Invalid command: {command}"
            )

    return converted