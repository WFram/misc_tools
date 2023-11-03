from tqdm import tqdm
import cv2
from os.path import join, exists
from os import makedirs
import numpy as np
import argparse
from typing import List


EXTENSION = '.jpg'
def export_images_kalibr(input_video: str, input_times : str, output_images: str) -> None:

    if not exists(output_images):
        makedirs(output_images)

    indices : List = []
    f_times = open(input_times, 'r')
    for time in f_times:
        indices.append(str(int(1e9 * float(time.split(' ')[1]))))
    f_times.close()

    capture = cv2.VideoCapture(input_video)

    if not capture.isOpened():
        print("Error opening video stream or file")

    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    pbar = tqdm(total=total_frames, unit=' frames')

    frame_id = 0
    while capture.isOpened():
        if len(indices) == frame_id:
            break

        status, frame = capture.read()
        if status:
            # TODO: add optional extension
            cv2.imwrite(join(output_images, indices[frame_id]) + EXTENSION, frame)
            frame_id += 1

            pbar.update(1)
        else:
            break

    pbar.close()
    capture.release()

    cv2.destroyAllWindows()


EXTENSION = '.jpg'
def export_images_colmap(input_video: str, output_images: str) -> None:

    capture = cv2.VideoCapture(input_video)

    if not capture.isOpened():
        print("Error opening video stream or file")

    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    pbar = tqdm(total=total_frames, unit=' frames')

    frame_id = 0
    while capture.isOpened():
        status, frame = capture.read()
        if status:
            # TODO: add optional extension
            cv2.imwrite(output_images + '{:06d}'.format(frame_id) + EXTENSION, frame)

            frame_id += 5
            pbar.update(1)
        else:
            break

    pbar.close()
    capture.release()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--input_video',
                        required=True,
                        help='path to input video')
    
    parser.add_argument('--input_times',
                        required=False,
                        help='path to input times')
    
    parser.add_argument('--output_images',
                        required=True,
                        help='path to folder with output images')
    
    parser.add_argument('--output_format',
                        required=False,
                        choices=['colmap', 'kalibr'],
                        default='colmap',
                        help='image name format')

    args = parser.parse_args()

    if args.output_format == 'colmap':
        export_images_colmap(args.input_video, args.output_images)
    elif args.output_format == 'kalibr':
        export_images_kalibr(args.input_video, args.input_times, args.output_images)