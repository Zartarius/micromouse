import cv2 as cv
import numpy as np
import math

class Node:
    def __init__(self, node_id, x, y):
        self.id = node_id
        self.x = x
        self.y = y

    def get_point(self):
        return (self.x,self.y)

    def get_ID(self):
        return self.id

class Graph:
    def __init__(self):
        self.nodes: dict[int, Node] = {} # Key: node_id, Value: Node object
        self.adj_list: dict[int, dict[int, int]] = {} # Key: node_id, Value: {node_id: weight} of neighbours

    def add_node(self, node_id, x, y):
        self.nodes[node_id] = Node(node_id, x, y)
        self.adj_list[node_id] = {}

    def add_edge(self, node_id1, node_id2, weight):
        self.adj_list[node_id1].update({node_id2: weight})
        self.adj_list[node_id2].update({node_id1: weight})

    def remove_edge(self, node_id1, node_id2):
        del self.adj_list[node_id1][node_id2]
        del self.adj_list[node_id2][node_id1]

    def get_nodes(self):
        return self.nodes

    def get_edge_weight(self, node_id1, node_id2):
        return self.adj_list[node_id1][node_id2]


def convert_image_to_bnw(image_path: str) -> cv.typing.MatLike:
    img = cv.imread(image_path)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    lowerb, upperb = np.array([0, 0, 164]), np.array([179, 255, 255])
    mask = cv.inRange(hsv, lowerb, upperb)

    return mask


def path_clear(image, x1, y1, x2, y2):
    num_points = int(max(abs(x2 - x1), abs(y2 - y1))) + 1

    xs = np.linspace(x1, x2, num_points)
    ys = np.linspace(y1, y2, num_points)

    for x, y in zip(xs, ys):
        px = int(round(x))
        py = int(round(y))

        if py < 0 or py >= image.shape[0] or px < 0 or px >= image.shape[1]:
            return False

        if np.any(image[py, px] != 255):
            return False

    return True


def build_bfs_graph(maze_image, n, start_node=None, end_node=None) -> tuple[Graph, cv.typing.Matlike]:
    image = maze_image.copy()
    graph = Graph()

    height, width, _ = image.shape
    v_dist = int(round(height / (n + 1)))
    h_dist = int(round(width / (n + 1)))

    # coords of all nodes
    node_coords = [[(h_dist + h_dist * c, v_dist + v_dist * r) for c in range(n)] for r in range(n)]

    for r in range(n):
        for c in range(n):
            cv.circle(image, node_coords[r][c], 3, [0, 255, 0], -1)
            node_id = r * n + c
            graph.add_node(node_id, *node_coords[r][c])

    if start_node is not None or end_node is not None:
        font = cv.FONT_HERSHEY_SIMPLEX
        if start_node is not None:
            start_coords = graph.nodes[start_node].get_point()
            cv.putText(image, str(start_node), start_coords, font, 1.0, (0, 255, 0), 1, cv.LINE_AA, bottomLeftOrigin=False)
        if end_node is not None:
            end_coords = graph.nodes[end_node].get_point()
            cv.putText(image, str(end_node), end_coords, font, 1.0, (0, 255, 0), 1, cv.LINE_AA, bottomLeftOrigin=False)

    for r in range(n):
        for c in range(n):
            right, down = None, None

            if c != n - 1:
                right = node_coords[r][c + 1]

            if r != n - 1:
                down = node_coords[r + 1][c]

            node_id1 = r * n + c

            if right and path_clear(maze_image, *node_coords[r][c], *right):
                cv.line(image, node_coords[r][c], right, (0, 125, 0), 1)
                graph.add_edge(node_id1, r * n + (c + 1), 1)

            if down and path_clear(maze_image, *node_coords[r][c], *down):
                cv.line(image, node_coords[r][c], down, (0, 125, 0), 1)
                graph.add_edge(node_id1, (r + 1) * n + c, 1)

    return graph, image