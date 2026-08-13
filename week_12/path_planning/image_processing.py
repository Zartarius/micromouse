import cv2
import numpy as np
import matplotlib.pyplot as plt


def find_colour_marker(hsv, lower, upper, offset=60):
    """
    Finds the largest coloured rectangle and returns

        centroid
        top-left
        bottom-right
        mask
    """

    mask = cv2.inRange(
        hsv,
        np.array(lower, dtype=np.uint8),
        np.array(upper, dtype=np.uint8)
    )

    # Remove small noise
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours_info = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # Compatible with OpenCV 3 and 4
    if len(contours_info) == 2:
        contours = contours_info[0]
    else:
        contours = contours_info[1]

    if len(contours) == 0:
        return None

    contour = max(contours, key=cv2.contourArea)

    M = cv2.moments(contour)

    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    top_left = (cx - offset, cy - offset)
    bottom_right = (cx + offset, cy + offset)

    return {
        "centroid": (cx, cy),
        "top_left": top_left,
        "bottom_right": bottom_right,
        "mask": mask
    }


def detect_corner_markers(image):

    hsv = cv2.cvtColor(
        image,
        cv2.COLOR_RGB2HSV
    )

    colours = {
        "top_left": {
            "lower": (140, 20, 130),
            "upper": (179, 41, 162)
        },

        "top_right": {
            "lower": (0, 135, 0),
            "upper": (179, 209, 255)
        },

        "bottom_left": {
            "lower": (109, 40, 100),
            "upper": (179, 255, 147)
        },

        "bottom_right": {
            "lower": (19, 91, 120),
            "upper": (34, 177, 255)
        }
    }

    centres = {}

    for name, values in colours.items():

        result = find_colour_marker(
            hsv,
            values["lower"],
            values["upper"]
        )

        if result is None:
            raise ValueError(
                f"Could not detect {name} marker."
            )

        centres[name] = result["centroid"]

    # ---------------------------------------------------------
    # Marker centroid -> actual maze corner
    #
    # TL: (+50, +50)
    # TR: (-50, +50)
    # BL: (+50, -50)
    # BR: (-50, -50)
    # ---------------------------------------------------------

    tl_x, tl_y = centres["top_left"]
    tr_x, tr_y = centres["top_right"]
    bl_x, bl_y = centres["bottom_left"]
    br_x, br_y = centres["bottom_right"]
    offset = 70
    source_points = np.float32([
        [tl_x - offset, tl_y - offset],   # top-left
        [tr_x + offset, tr_y - offset],   # top-right
        [bl_x - (offset+30), bl_y + (offset-20)],   # bottom-left
        [br_x + offset, br_y + offset]    # bottom-right
    ])

    return source_points

def load_and_process_image(image_file, side=900):
    """
    Load the camera image, apply perspective correction and convert
    the maze into a binary occupancy image.
    """

    image = cv2.imread(image_file)

    if image is None:
        raise FileNotFoundError(f"Could not load image: {image_file}")

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Map the four maze corners to a square so that each cell has
    # approximately equal dimensions for wall detection.
    source_points = detect_corner_markers(image)

    display = image.copy()

    # Draw detected maze corner points
    for point in source_points:

        x, y = point.astype(int)

        cv2.circle(
            display,
            (x, y),
            5,
            (0, 255, 0),
            -1
        )

    # Connect the four corners
    for i in range(4):

        p1 = tuple(
            source_points[i].astype(int)
        )

        p2 = tuple(
            source_points[(i + 1) % 4].astype(int)
        )

        cv2.line(
            display,
            p1,
            p2,
            (255, 0, 0),
            2
        )

    plt.figure(figsize=(8, 8))

    plt.imshow(display)

    plt.title("Detected Maze Corners")

    plt.show()

    destination_points = np.float32([
        [0, 0],
        [side, 0],
        [0, side],
        [side, side]
    ])

    transform = cv2.getPerspectiveTransform(
        source_points,
        destination_points
    )

    image = cv2.warpPerspective(
        image,
        transform,
        (side, side)
    )

    # Threshold the image in HSV space to isolate the maze walls.
    lower_black = (0, 0, 0)
    upper_black = (180, 34, 110)

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Invert so that walls are represented by zero-valued pixels.
    binary = cv2.bitwise_not(mask)

    return binary


def split_into_cells(binary_image, grid_size):
    """
    Divide the binary maze image into individual grid cells.
    """

    height, width = binary_image.shape

    cell_height = height // grid_size
    cell_width = width // grid_size

    cells = []

    for row in range(grid_size):
        row_cells = []

        for col in range(grid_size):
            y1 = row * cell_height
            y2 = (row + 1) * cell_height

            x1 = col * cell_width
            x2 = (col + 1) * cell_width

            cell = binary_image[y1:y2, x1:x2]
            row_cells.append(cell)

        cells.append(row_cells)

    return cells, cell_width, cell_height


def check_walls(cell, threshold=0.5):
    """
    Detect the four walls surrounding a maze cell.

    The outer 30% of each side is examined. A wall is detected when
    a sufficiently large proportion of pixels in one row or column
    is classified as black.
    """

    height, width = cell.shape

    vertical_region = int(0.30 * width)
    horizontal_region = int(0.30 * height)

    # East wall
    east_region = cell[:, width - vertical_region:]
    east_black = np.mean(east_region == 0, axis=0)
    east_wall = np.max(east_black) > threshold

    # West wall
    west_region = cell[:, :vertical_region]
    west_black = np.mean(west_region == 0, axis=0)
    west_wall = np.max(west_black) > threshold

    # North wall
    north_region = cell[:horizontal_region, :]
    north_black = np.mean(north_region == 0, axis=1)
    north_wall = np.max(north_black) > threshold

    # South wall
    south_region = cell[height - horizontal_region:, :]
    south_black = np.mean(south_region == 0, axis=1)
    south_wall = np.max(south_black) > threshold

    return {
        "N": north_wall,
        "S": south_wall,
        "E": east_wall,
        "W": west_wall
    }


def detect_maze_walls(cells, grid_size):
    """
    Detect walls for every cell in the maze.
    """

    maze = []

    for row in range(grid_size):
        maze_row = []

        for col in range(grid_size):
            walls = check_walls(cells[row][col])
            maze_row.append(walls)

        maze.append(maze_row)

    return maze


def synchronise_walls(graph, maze, grid_size):
    """
    Ensure shared walls are consistent between neighbouring cells.

    If either cell detects a wall between them, both cells are marked
    as having that wall.
    """

    for row in range(grid_size):
        for col in range(grid_size):

            node = graph.nodes[row * grid_size + col]
            walls = maze[row][col]

            node.north = walls["N"]
            node.south = walls["S"]
            node.east = walls["E"]
            node.west = walls["W"]

    # Synchronise east/west walls.
    for row in range(grid_size):
        for col in range(grid_size - 1):

            node = graph.nodes[row * grid_size + col]
            east_node = graph.nodes[row * grid_size + col + 1]

            wall_exists = node.east or east_node.west

            node.east = wall_exists
            east_node.west = wall_exists

    # Synchronise north/south walls.
    for row in range(grid_size - 1):
        for col in range(grid_size):

            node = graph.nodes[row * grid_size + col]
            south_node = graph.nodes[(row + 1) * grid_size + col]

            wall_exists = node.south or south_node.north

            node.south = wall_exists
            south_node.north = wall_exists


def display_occupancy_map(binary, cell_width, cell_height, grid_size):
    """
    Display the processed occupancy map with the maze grid overlaid.
    """

    height, width = binary.shape

    plt.figure(figsize=(8, 8))
    plt.imshow(binary, cmap="gray")

    for i in range(1, grid_size):
        plt.axhline(
            i * cell_height,
            color="red",
            linewidth=1
        )

        plt.axvline(
            i * cell_width,
            color="red",
            linewidth=1
        )

    plt.xlim(0, width)
    plt.ylim(height, 0)
    plt.title(f"{grid_size} × {grid_size} Maze Grid")
    plt.show()