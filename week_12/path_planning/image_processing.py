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
            "lower": (140, 30, 130),
            "upper": (179, 60, 165)
        },

        "top_right": {
            "lower": (0, 135, 134),
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
    offset = 60
    source_points = np.float32([
        [tl_x - offset, tl_y - offset],   # top-left
        [tr_x + offset, tr_y - offset],   # top-right
        [bl_x - (offset), bl_y + (offset)],   # bottom-left
        [br_x + offset, br_y + offset]    # bottom-right
    ])

    return source_points

def detect_corner_markers_manual(image):

    # ---------------------------------------------------------
    # Resize image ONLY for display
    # ---------------------------------------------------------

    max_display_size = 900

    h, w = image.shape[:2]

    scale = min(
        max_display_size / w,
        max_display_size / h,
        1.0
    )

    display_w = int(w * scale)
    display_h = int(h * scale)

    display = cv2.resize(
        image,
        (display_w, display_h),
        interpolation=cv2.INTER_AREA
    )

    # RGB -> BGR for OpenCV
    display = cv2.cvtColor(
        display,
        cv2.COLOR_RGB2BGR
    )

    points = []

    window_name = "Select Maze Corners"

    cv2.namedWindow(
        window_name,
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        window_name,
        display_w,
        display_h
    )

    # ---------------------------------------------------------
    # Mouse callback
    # ---------------------------------------------------------

    def mouse_callback(event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:

            if len(points) >= 4:
                return

            # Convert display coordinates
            # back to original image coordinates
            original_x = int(x / scale)
            original_y = int(y / scale)

            points.append(
                (original_x, original_y)
            )

            # Draw point
            cv2.circle(
                display,
                (x, y),
                6,
                (0, 255, 0),
                -1
            )

            # Number point
            cv2.putText(
                display,
                str(len(points)),
                (x + 10, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            print(
                f"Point {len(points)}: "
                f"({original_x}, {original_y})"
            )

    cv2.setMouseCallback(
        window_name,
        mouse_callback
    )

    print("Click:")
    print("1 = Top-left")
    print("2 = Top-right")
    print("3 = Bottom-left")
    print("4 = Bottom-right")
    print("ESC = Cancel")

    # ---------------------------------------------------------
    # Display loop
    # ---------------------------------------------------------

    while len(points) < 4:

        cv2.imshow(
            window_name,
            display
        )

        key = cv2.waitKey(20) & 0xFF

        if key == 27:
            cv2.destroyAllWindows()

            raise ValueError(
                "Manual corner selection cancelled."
            )

    # ---------------------------------------------------------
    # Fourth point has been selected
    # ---------------------------------------------------------

    cv2.imshow(
        window_name,
        display
    )

    # Give OpenCV time to display the fourth point
    cv2.waitKey(300)

    cv2.destroyAllWindows()

    # ---------------------------------------------------------
    # Return same format as detect_corner_markers()
    # ---------------------------------------------------------

    source_points = np.float32([
        points[0],
        points[1],
        points[2],
        points[3]
    ])

    return source_points

def load_and_process_image(image_file, side=1620):
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
    border = 10

    cv2.rectangle(image, (0, 0), (side - 1, side - 1), (0, 0, 0), border)

    lower_black = (0, 0, 140)
    upper_black = (179, 255, 200)

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Invert so that walls are represented by zero-valued pixels.
    binary = mask

    return binary, image

def process_image(image):
    """
    Apply an HSV mask to isolate the maze walls and return a binary
    occupancy image.

    Input:
        image - RGB image

    Output:
        binary image (uint8), walls represented by zero-valued pixels
    """

    # Threshold the image in HSV space to isolate the maze walls.
    lower_black = (0, 0, 150)
    upper_black = (120, 40, 200)

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_black, upper_black)

    # Invert so that walls are represented by zero-valued pixels.
    binary = mask

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

def draw_path_on_image(image, path, rows=9, cols=9):
    """
    Draw the calculated maze path on the perspective-transformed image.
    """

    output = image.copy()

    height, width = output.shape[:2]

    cell_width = width / cols
    cell_height = height / rows

    # ---------------------------------------------------------
    # Calculate centre of each node
    # ---------------------------------------------------------

    points = []

    for node_id in path:

        row = node_id // cols
        col = node_id % cols

        x = int((col + 0.5) * cell_width)
        y = int((row + 0.5) * cell_height)

        points.append((x, y))

    # ---------------------------------------------------------
    # Draw path
    # ---------------------------------------------------------

    for i in range(len(points) - 1):

        cv2.line(
            output,
            points[i],
            points[i + 1],
            (255, 0, 0),
            5
        )

    # ---------------------------------------------------------
    # Draw start and goal
    # ---------------------------------------------------------

    cv2.circle(
        output,
        points[0],
        10,
        (0, 255, 0),
        -1
    )

    cv2.circle(
        output,
        points[-1],
        10,
        (255, 0, 0),
        -1
    )

    return output

def make_cells_black(binary, cell_ids, rows=9, cols=9):
    """
    Force selected cells in the binary maze image to completely black.
    """

    height, width = binary.shape[:2]

    cell_height = height // rows
    cell_width = width // cols

    for node_id in cell_ids:

        row = node_id // cols
        col = node_id % cols

        y1 = row * cell_height
        y2 = (row + 1) * cell_height

        x1 = col * cell_width
        x2 = (col + 1) * cell_width

        binary[y1:y2, x1:x2] = 0

    return binary

def extract_obstacle_area(transformed_image):
    """
    Extract the fixed 5x5 obstacle area.

    Top-left internal cell     : (3, 1)
    Bottom-right internal cell: (7, 5)
    """

    height, width = transformed_image.shape[:2]

    cell_width = width / 9
    cell_height = height / 9

    x1 = int(3 * cell_width)
    y1 = int(1 * cell_height)

    x2 = int(8 * cell_width)
    y2 = int(6 * cell_height)

    return transformed_image[y1:y2, x1:x2]

def create_obstacle_occupancy_map(
    binary_image,
    graph,
    rows,
    cols,
    top_left,
    obstacle_cells=5,
    obstacle_buffer=40,
    wall_buffer=45,
    min_area=500,
    edge_margin=50,
    output_size=900
):
    """
    Create a pixel-level occupancy map for the obstacle course.

    0 = free space
    1 = obstacle / buffered obstacle

    The obstacle course is defined by a square block of
    `obstacle_cells x obstacle_cells` cells starting at `top_left`.

    Parameters
    ----------
    binary_image : numpy.ndarray
        Original binary/transformed image.

    graph : dict
        Maze graph containing the detected wall information.

    rows, cols : int
        Number of maze rows and columns.

    top_left : tuple
        (row, col) of the top-left obstacle-course cell.

    obstacle_cells : int
        Number of cells forming the obstacle course.

    obstacle_buffer : int
        Safety buffer around detected physical obstacles.

    wall_buffer : int
        Safety buffer around maze walls.

    min_area : int
        Minimum contour area to be considered an obstacle.

    edge_margin : int
        Additional margin around the boundary of the obstacle course.

    output_size : int
        Final square occupancy-map size in pixels.
    """

    # ---------------------------------------------------------
    # 1. Image dimensions
    # ---------------------------------------------------------

    height, width = binary_image.shape[:2]

    # ---------------------------------------------------------
    # 2. Maze cell dimensions
    # ---------------------------------------------------------

    cell_width = width / cols
    cell_height = height / rows

    # ---------------------------------------------------------
    # 3. Obstacle-course boundaries
    # ---------------------------------------------------------

    top_row, left_col = top_left

    x1 = int(left_col * cell_width)
    y1 = int(top_row * cell_height)

    x2 = int((left_col + obstacle_cells) * cell_width)
    y2 = int((top_row + obstacle_cells) * cell_height)

    # ---------------------------------------------------------
    # 4. Crop obstacle course
    # ---------------------------------------------------------

    obstacle_area = binary_image[y1:y2, x1:x2].copy()

    # ---------------------------------------------------------
    # 5. Resize to requested output size
    # ---------------------------------------------------------

    obstacle_area = cv2.resize(
        obstacle_area,
        (output_size, output_size),
        interpolation=cv2.INTER_NEAREST
    )

    # ---------------------------------------------------------
    # 6. Convert to grayscale if necessary
    # ---------------------------------------------------------

    if len(obstacle_area.shape) == 3:

        gray = cv2.cvtColor(
            obstacle_area,
            cv2.COLOR_BGR2GRAY
        )

    else:

        gray = obstacle_area.copy()

    # ---------------------------------------------------------
    # 7. Detect physical obstacles
    # ---------------------------------------------------------

    obstacle_mask = np.zeros_like(
        gray,
        dtype=np.uint8
    )

    obstacle_mask[gray < 100] = 255

    # ---------------------------------------------------------
    # 8. Find obstacle contours
    # ---------------------------------------------------------

    contours, _ = cv2.findContours(
        obstacle_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # ---------------------------------------------------------
    # 9. Create empty occupancy map
    # ---------------------------------------------------------

    occupancy_map = np.zeros(
        (output_size, output_size),
        dtype=np.uint8
    )

    # ---------------------------------------------------------
    # 10. Keep only sufficiently large obstacles
    # ---------------------------------------------------------

    for contour in contours:

        area = cv2.contourArea(contour)

        if area >= min_area:

            cv2.drawContours(
                occupancy_map,
                [contour],
                -1,
                1,
                thickness=cv2.FILLED
            )

    # ---------------------------------------------------------
    # 11. Buffer physical obstacles
    # ---------------------------------------------------------

    if obstacle_buffer > 0:

        kernel_size = (
            2 * obstacle_buffer + 1
        )

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size)
        )

        obstacle_binary = (
            occupancy_map * 255
        ).astype(np.uint8)

        obstacle_binary = cv2.dilate(
            obstacle_binary,
            kernel,
            iterations=1
        )

        occupancy_map[
            obstacle_binary > 0
        ] = 1

    # ---------------------------------------------------------
    # 12. Add boundary / edge margin
    # ---------------------------------------------------------

    if edge_margin > 0:

        occupancy_map[
            :edge_margin,
            :
        ] = 1

        occupancy_map[
            -edge_margin:,
            :
        ] = 1

        occupancy_map[
            :,
            :edge_margin
        ] = 1

        occupancy_map[
            :,
            -edge_margin:
        ] = 1

    # ---------------------------------------------------------
    # 13. Return 0/1 occupancy map
    # ---------------------------------------------------------

    occupancy_map = (
        occupancy_map > 0
    ).astype(np.uint8)

    return occupancy_map

def display_full_maze_with_occupancy(
    transformed_image,
    occupancy_map
):
    """
    Display the complete perspective-transformed maze,
    with only the fixed 5x5 obstacle area replaced by
    its occupancy map.

    0 = free
    1 = obstacle
    """

    # ---------------------------------------------------------
    # Copy original image
    # ---------------------------------------------------------

    output = transformed_image.copy()

    height, width = output.shape[:2]

    # ---------------------------------------------------------
    # 9x9 cell dimensions
    # ---------------------------------------------------------

    cell_width = width / 9
    cell_height = height / 9

    # ---------------------------------------------------------
    # Fixed 5x5 obstacle area
    #
    # Top-left     = (3,1)
    # Bottom-right = (7,5)
    # ---------------------------------------------------------

    x1 = int(3 * cell_width)
    y1 = int(1 * cell_height)

    x2 = int(8 * cell_width)
    y2 = int(6 * cell_height)

    # ---------------------------------------------------------
    # Resize occupancy map to exactly fit obstacle region
    # ---------------------------------------------------------

    occupancy_resized = cv2.resize(
        occupancy_map,
        (x2 - x1, y2 - y1),
        interpolation=cv2.INTER_NEAREST
    )

    # ---------------------------------------------------------
    # Convert occupancy map to display image
    #
    # Free     = white
    # Obstacle = black
    # ---------------------------------------------------------

    occupancy_display = np.ones(
        (
            y2 - y1,
            x2 - x1,
            3
        ),
        dtype=np.uint8
    ) * 255

    occupancy_display[
        occupancy_resized == 1
    ] = [0, 0, 0]

    # ---------------------------------------------------------
    # Replace ONLY the obstacle area
    # ---------------------------------------------------------

    output[
        y1:y2,
        x1:x2
    ] = occupancy_display

    return output


def add_maze_walls_to_occupancy(
    occupancy_map,
    graph,
    rows=9,
    cols=9,
    top_left=(3, 1),
    bottom_right=(7, 5),
    wall_thickness=10,
    wall_buffer=90
):
    """
    Add maze walls surrounding the 5x5 obstacle area
    to the occupancy map.

    Existing obstacle occupancy:
        0 = free
        1 = occupied

    Added:
        maze walls + 90 pixel safety buffer

    The occupancy_map is modified and returned.
    """

    height, width = occupancy_map.shape[:2]

    # ---------------------------------------------------------
    # Size of each internal cell
    # ---------------------------------------------------------

    obstacle_width = (
        bottom_right[0] - top_left[0] + 1
    )

    obstacle_height = (
        bottom_right[1] - top_left[1] + 1
    )

    cell_width = width / obstacle_width
    cell_height = height / obstacle_height

    # ---------------------------------------------------------
    # Copy occupancy map
    # ---------------------------------------------------------

    result = occupancy_map.copy()

    # ---------------------------------------------------------
    # Helper: add buffered wall
    # ---------------------------------------------------------

    def add_vertical_wall(x):

        x = int(x)

        # Wall itself
        half = wall_thickness // 2

        x1 = max(0, x - half)
        x2 = min(width, x + half + 1)

        result[:, x1:x2] = 1

        # Buffer
        buffer_x1 = max(
            0,
            x - half - wall_buffer
        )

        buffer_x2 = min(
            width,
            x + half + wall_buffer + 1
        )

        result[:, buffer_x1:buffer_x2] = 1


    def add_horizontal_wall(y):

        y = int(y)

        half = wall_thickness // 2

        y1 = max(0, y - half)
        y2 = min(height, y + half + 1)

        result[y1:y2, :] = 1

        # Buffer
        buffer_y1 = max(
            0,
            y - half - wall_buffer
        )

        buffer_y2 = min(
            height,
            y + half + wall_buffer + 1
        )

        result[buffer_y1:buffer_y2, :] = 1

    # ---------------------------------------------------------
    # Check walls around the obstacle region
    # ---------------------------------------------------------

    # We only need the walls forming boundaries between
    # the obstacle area and the normal maze.

    left_x = 0
    right_x = width

    top_y = 0
    bottom_y = height

    # ---------------------------------------------------------
    # LEFT boundary
    # ---------------------------------------------------------

    # Cells on left side of obstacle region:
    # x = 2, y = 1..5

    for y in range(
        top_left[1],
        bottom_right[1] + 1
    ):

        node_id = y * cols + (top_left[0] - 1)

        if graph.nodes[node_id].east:

            wall_y1 = (
                y - top_left[1]
            ) * cell_height

            wall_y2 = (
                y - top_left[1] + 1
            ) * cell_height

            x = 0

            result[
                int(wall_y1):int(wall_y2),
                0:min(width, wall_buffer)
            ] = 1


    # ---------------------------------------------------------
    # RIGHT boundary
    # ---------------------------------------------------------

    # Cells on right side:
    # x = 8, y = 1..5

    for y in range(
        top_left[1],
        bottom_right[1] + 1
    ):

        node_id = y * cols + (
            bottom_right[0] + 1
        )

        if graph.nodes[node_id].west:

            wall_y1 = (
                y - top_left[1]
            ) * cell_height

            wall_y2 = (
                y - top_left[1] + 1
            ) * cell_height

            result[
                int(wall_y1):int(wall_y2),
                max(0, width - wall_buffer):width
            ] = 1


    # ---------------------------------------------------------
    # TOP boundary
    # ---------------------------------------------------------

    # Cells above obstacle region:
    # y = 0, x = 3..7

    for x in range(
        top_left[0],
        bottom_right[0] + 1
    ):

        node_id = (
            (top_left[1] - 1) * cols
            + x
        )

        if graph.nodes[node_id].south:

            wall_x1 = (
                x - top_left[0]
            ) * cell_width

            wall_x2 = (
                x - top_left[0] + 1
            ) * cell_width

            result[
                0:min(height, wall_buffer),
                int(wall_x1):int(wall_x2)
            ] = 1


    # ---------------------------------------------------------
    # BOTTOM boundary
    # ---------------------------------------------------------

    # Cells below obstacle region:
    # y = 6, x = 3..7

    for x in range(
        top_left[0],
        bottom_right[0] + 1
    ):

        node_id = (
            (bottom_right[1] + 1) * cols
            + x
        )

        if graph.nodes[node_id].north:

            wall_x1 = (
                x - top_left[0]
            ) * cell_width

            wall_x2 = (
                x - top_left[0] + 1
            ) * cell_width

            result[
                max(0, height - wall_buffer):height,
                int(wall_x1):int(wall_x2)
            ] = 1

    return result