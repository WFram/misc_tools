import os
import sys
import argparse

import cv2
import numpy as np

import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def get_image_files(path):
    return sorted([os.path.join(path, f) for f in os.listdir(path) if f.endswith('.png') or f.endswith('.jpg')])


def read_image_file(path):
    return cv2.imread(path)


def downscale_image(input_image):
    scale_percent = 50

    width = int(input_image.shape[1] * scale_percent / 100)
    height = int(input_image.shape[0] * scale_percent / 100)
    dim = (width, height)

    output_image = cv2.resize(input_image, dim, interpolation=cv2.INTER_AREA)

    return output_image

# TODO: generalize
def crop_image(input_image):
    crop_width = 0
    crop_height = 100
    output_image = input_image[:input_image.shape[0] - crop_height, :input_image.shape[1] - crop_width, :]
    return output_image


# NOTE: Assumed that the naming for .png files correspond to their timestamps
def create_rosbag(args):
    # TODO: remove
    rospy.init_node('image_converter')

    image_files_left = get_image_files(args.path_to_left_images)
    image_files_right = get_image_files(args.path_to_right_images)
    output_bag_file = args.output_bag_file
    times_from_filenames = args.times_from_filenames
    times_from_text_file = args.times_from_text_file
    path_to_times = args.path_to_times
    downscale = args.downscale
    crop = args.crop

    if len(image_files_left) != len(image_files_right):
        print('Error: The number of left and right images is not the same')
        sys.exit(1)

    bridge = CvBridge()

    # TODO: generalize (for now it suits only 4seasons; TUM format)
    times = np.zeros([len(image_files_left), 3])

    if times_from_text_file:
        times = np.loadtxt(path_to_times, delimiter=' ')

    frame_counter = 0

    # TODO: add some bars
    TIMESTAMP_FACTOR = 1
    with rosbag.Bag(output_bag_file, 'w') as bag:
        for i in range(len(image_files_left)):
            if times_from_filenames:
                time_usec_left = float(os.path.basename(image_files_left[i]).replace(".png", ""))
                time_usec_right = float(os.path.basename(image_files_right[i]).replace(".png", ""))
                time_stamp_left = rospy.Time.from_sec(time_usec_left / TIMESTAMP_FACTOR)
                time_stamp_right = rospy.Time.from_sec(time_usec_right / TIMESTAMP_FACTOR)
            elif times_from_text_file:
                time_stamp_left = rospy.Time.from_sec(times[frame_counter][1] / TIMESTAMP_FACTOR)
                time_stamp_right = rospy.Time.from_sec(times[frame_counter][1] / TIMESTAMP_FACTOR)
            else:
                time_stamp_left = rospy.Time.now()
                time_stamp_right = time_stamp_left

            img_left = read_image_file(image_files_left[i])
            img_right = read_image_file(image_files_right[i])

            if downscale:
                img_left = downscale_image(img_left)
                img_right = downscale_image(img_right)

            if crop:
                img_left = crop_image(img_left)
                img_right = crop_image(img_right)

            msg_left = bridge.cv2_to_imgmsg(img_left, encoding="bgr8")
            msg_right = bridge.cv2_to_imgmsg(img_right, encoding="bgr8")

            msg_left.header.stamp = time_stamp_left
            msg_right.header.stamp = time_stamp_right

            # TODO: make configurable
            msg_left.header.frame_id = "camera_left"
            msg_right.header.frame_id = "camera_right"

            # TODO: make configurable
            bag.write("/camera/left/image_raw", msg_left, time_stamp_left)
            bag.write("/camera/right/image_raw", msg_right, time_stamp_right)

            frame_counter += 1

    print('Conversion completed successfully')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_left_images', type=str)
    parser.add_argument('--path_to_right_images', type=str)
    parser.add_argument('--output_bag_file', type=str)
    parser.add_argument('--times_from_filenames', action='store_true')
    parser.add_argument('--times_from_text_file', action='store_true')
    parser.add_argument('--path_to_times', type=str)
    parser.add_argument('--downscale', action='store_true')
    parser.add_argument('--crop', action='store_true')
    args = parser.parse_args()

    create_rosbag(args)
