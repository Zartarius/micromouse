import sys
import cv2 as cv
import path_planning.path_finder as pg

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"usage: python3 {sys.argv[0]} <image_path> [output_size] [output_path]")
        sys.exit(1)

    image_path = sys.argv[1]
    output_size = int(sys.argv[2]) if len(sys.argv) > 2 else 600
    output_path = sys.argv[3] if len(sys.argv) > 3 else "maze_images/pg_test_square.jpg"

    square_img = pg.perspective_transform_to_square(image_path, output_size)
    cv.imwrite(output_path, square_img)
    print(f"saved {output_path}")
