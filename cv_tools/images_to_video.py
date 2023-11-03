import cv2 as cv
import os
from tqdm import tqdm

import argparse


EXTENSION = ".jpg"
def images_to_video(image_folder: str, video_name: str, fps: int):
    # TODO: make an extension optional
    images = [img for img in os.listdir(image_folder) if img.endswith(EXTENSION)]
    images.sort()
    frame = cv.imread(os.path.join(image_folder, images[0]))
    height, width, _ = frame.shape

    pbar = tqdm(total=len(images), unit=' frames')

    OUTPUT_VIDEO_FORMAT_FOURCC = cv.VideoWriter_fourcc(*'mp4v')
    video = cv.VideoWriter(video_name, OUTPUT_VIDEO_FORMAT_FOURCC, fps, (width, height))

    for image in images:
        video.write(cv.imread(os.path.join(image_folder, image)))
        pbar.update(1)

    pbar.close()
    cv.destroyAllWindows()
    video.release()


if __name__ == '__main__':

    parser = argparse.ArgumentParser()

    parser.add_argument('--input_folder',
                        type=str,
                        required=True,
                        help='folder with images')
    
    parser.add_argument('--output_video',
                        required=True,
                        type=str,
                        help='video file name')
    
    parser.add_argument('--fps',
                        required=False,
                        type=int,
                        default=30,
                        help='fps of the output video')
    
    args = parser.parse_args()

    image_folder = args.input_folder
    video_name = os.path.join(image_folder, args.output_video) 
    fps = args.fps

    images_to_video(image_folder, video_name, fps)