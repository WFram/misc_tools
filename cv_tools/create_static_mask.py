import argparse
import cv2 as cv
import numpy as np


def create_mask(args) -> None:
    input_image = cv.imread(args.path_to_input)

    mask = np.zeros_like(input_image[:, :, 0])

    THICKNESS = 90
    def draw_mask(event, x, y, flags, parameters) -> None:
        mode = 255
        drawing = False

        opencv_coordinates = (x * 2, y * 2)

        if event == cv.EVENT_LBUTTONDOWN:
            drawing = True
            cv.circle(mask, opencv_coordinates, THICKNESS, mode, -1)

        elif event == cv.EVENT_MOUSEMOVE:
            if drawing:
                cv.circle(mask, opencv_coordinates, THICKNESS, mode, -1)

        elif event == cv.EVENT_LBUTTONUP:
            drawing = False
            cv.circle(mask, opencv_coordinates, THICKNESS, mode, -1)

    window_name = 'GUI'
    cv.namedWindow(window_name)
    cv.setMouseCallback(window_name, draw_mask)

    while True:
        masked_img = cv.addWeighted(input_image, 0.7, cv.cvtColor(mask, cv.COLOR_GRAY2BGR), 0.3, 0)
        cv.imshow(window_name, cv.resize(masked_img, (0, 0), fx=0.5, fy=0.5))

        k = cv.waitKey(1) & 0xFF

        if k == ord('q'):
            mask = cv.bitwise_not(mask)
            cv.imwrite(args.path_to_output, mask)
            break

        elif k == ord('c'):
            mask = np.zeros_like(input_image[:, :, 0])

    cv.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="GUI interface for masking selected image area \
                                     Mouse click to draw; 'q' to save a mask and finish")
    parser.add_argument('--path_to_input', type=str,
                        help="Input image")
    parser.add_argument('--path_to_output', type=str,
                        help="Output mask")
    args = parser.parse_args()

    create_mask(args)