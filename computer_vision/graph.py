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

            node_id = row * grid_size + col

            graph.add_node(
                node_id,
                col,
                row
            )

    return graph


def add_valid_edges(graph, grid_size):
    """
    Connect adjacent cells that do not have a wall between them.
    """

    for row in range(grid_size):
        for col in range(grid_size):

            node_id = row * grid_size + col
            node = graph.nodes[node_id]

            # Connect to the eastern cell when no east wall exists.
            if col < grid_size - 1:

                east_node_id = row * grid_size + col + 1

                if not node.east:
                    graph.add_edge(
                        node_id,
                        east_node_id,
                        1
                    )

            # Connect to the southern cell when no south wall exists.
            if row < grid_size - 1:

                south_node_id = (row + 1) * grid_size + col

                if not node.south:
                    graph.add_edge(
                        node_id,
                        south_node_id,
                        1
                    )


def print_maze(graph, rows=9, cols=9):
    """
    Print an ASCII representation of the detected maze.
    """

    print("+" + "---+" * cols)

    for row in range(rows):

        line = ""

        for col in range(cols):

            node_id = row * cols + col
            node = graph.nodes[node_id]

            line += "|" if node.west else " "
            line += "   "

        node = graph.nodes[row * cols + cols - 1]
        line += "|" if node.east else " "

        print(line)

        line = ""

        for col in range(cols):

            node_id = row * cols + col
            node = graph.nodes[node_id]

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

    # Top boundary
    print("+" + "---+" * cols)

    for row in range(rows):

        line = ""

        for col in range(cols):

            node_id = row * cols + col
            node = graph.nodes[node_id]

            # West wall
            line += "|" if node.west else " "

            # Cell contents
            if node_id == start:
                line += " S "
            elif node_id == goal:
                line += " G "
            elif node_id in path_set:
                line += " • "
            else:
                line += "   "

        # East wall
        node = graph.nodes[row * cols + (cols - 1)]
        line += "|" if node.east else " "

        print(line)

        # Horizontal walls
        line = ""

        for col in range(cols):

            node_id = row * cols + col
            node = graph.nodes[node_id]

            line += "+"
            line += "---" if node.south else "   "

        line += "+"
        print(line)