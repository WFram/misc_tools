#!/usr/bin/env python

import argparse
import cv2 as cv


def get_colors(args) -> None:

    image = cv.imread(args.path_to_image)
    cv.imshow("Image", image)

    def get_color(event, x, y, flags, parameters) -> None:
        if event == cv.EVENT_LBUTTONDOWN:
            color = image[y, x]
            b, g, r = color[0], color[1], color[2]
            print(f"COLOR (bgr): {b, g, r}")

    cv.setMouseCallback("Image", get_color)
    cv.waitKey(0)

    cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Get color of a selected image pixel using GUI")
    parser.add_argument('--path_to_image', type=str)
    args = parser.parse_args()

    get_colors(args)
