#!/usr/bin/env python

import argparse
import cv2 as cv
import numpy as np
import os


def get_image_files(path):
    return sorted([os.path.join(path, f) for f in os.listdir(path) if f.endswith('.png') or f.endswith('.jpg')])


def crop(input_image_path, y0, y1, x0, x1):
    input_image = cv.imread(input_image_path)

    output_image = input_image[y0:y1, x0:x1, :]

    return output_image


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode',
                        required=True,
                        choices=['sample', 'folder'])
    parser.add_argument('--raw_sample_image',
                        required=False,
                        type=str)
    parser.add_argument('--raw_image_folder',
                        required=False,
                        type=str)
    parser.add_argument('--cropped_image_folder',
                        required=False,
                        type=str)
    args = parser.parse_args()

    if args.mode == 'folder':
        cropped_image_folder = args.cropped_image_folder
        if not os.path.exists(cropped_image_folder):
            os.makedirs(cropped_image_folder)
            print(f'Folder {cropped_image_folder} created successfully')

        image_files = get_image_files(args.raw_image_folder)
        Y0 = 0
        Y1 = 500
        X0 = 0
        X1 = 500
        for i in range(len(image_files)):
            output_image = crop(image_files[i], Y0, Y1, X0, X1)
            cv.imwrite(os.path.join(cropped_image_folder, os.path.basename(image_files[i])), output_image)

    if args.mode == 'sample':
        output_image = crop(args.raw_sample_image, Y0, Y1, X0, X1)
        cv.imshow("test", output_image)
        cv.waitKey(0)
