#!/usr/bin/env python

import argparse
import cv2 as cv
import os


def get_image_files(path):
    return sorted([os.path.join(path, f) for f in os.listdir(path) if f.endswith('.png') or f.endswith('.jpg')])


def put_mask(sample_image_filename, mask_filename):
    input_image = cv.imread(sample_image_filename)
    mask = cv.imread(mask_filename, 0)
    mask = cv.bitwise_not(mask)

    output_image = cv.bitwise_or(input_image, input_image, mask=mask)

    return output_image


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Put a given mask on provided set of images")
    parser.add_argument('--mode',
                        required=True,
                        choices=['sample', 'folder'])
    parser.add_argument('--mask',
                        required=True,
                        type=str)
    parser.add_argument('--raw_sample_image',
                        required=False,
                        type=str)
    parser.add_argument('--raw_image_folder',
                        required=False,
                        type=str)
    parser.add_argument('--masked_image_folder',
                        required=False,
                        type=str)
    args = parser.parse_args()

    if args.mode == 'folder':
        masked_image_folder = args.masked_image_folder
        if not os.path.exists(masked_image_folder):
            os.makedirs(masked_image_folder)
            print(f'Folder {masked_image_folder} created successfully')

        image_files = get_image_files(args.raw_image_folder)
        for i in range(len(image_files)):
            output_image = put_mask(image_files[i], args.mask)
            cv.imwrite(os.path.join(masked_image_folder, os.path.basename(image_files[i])), output_image)

    if args.mode == 'sample':
        output_image = put_mask(args.raw_sample_image, args.mask)
        cv.imshow("test", output_image)
        cv.waitKey(0)
