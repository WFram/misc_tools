from glob import glob
import os
import argparse

import rosbag, rospy
from cv_bridge import CvBridge

import cv2


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


def convert(args):
    rospy.init_node('image_converter')

    image_files = get_image_files(args.path_to_images)
    images_topic = args.images_topic
    output_bag_file = args.output_bag_file
    times_from_filenames = args.times_from_filenames
    frames_to_skip = args.frames_to_skip
    downscale = args.downscale

    if frames_to_skip != 0:
        print(f"{frames_to_skip} will be skipped")

    bridge = CvBridge()

    frame_count = 0

    # TODO: add some bars
    IMAGE_EXTENSION = ".png"
    TIMESTAMP_FACTOR = 1
    with rosbag.Bag(output_bag_file, 'w') as bag:
        for i in range(len(image_files)):
            if frame_count % (frames_to_skip + 1) != 0:
                frame_count += 1
                continue
            timestamp = rospy.Time.now()
            if times_from_filenames:
                time_usec = float(os.path.basename(image_files[i]).replace(IMAGE_EXTENSION, ""))
                timestamp = rospy.Time.from_sec(time_usec / TIMESTAMP_FACTOR)

            img = read_image_file(image_files[i])

            if downscale:
                img = downscale_image(img)

            msg = bridge.cv2_to_imgmsg(img, encoding="rgb8")

            msg.header.stamp = timestamp

            # TODO: make configurable
            msg.header.frame_id = "camera_left"

            # TODO: make configurable
            bag.write(images_topic, msg, timestamp)

            frame_count += 1

    print('Conversion completed successfully')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_images", help="Path to the folder with images")
    parser.add_argument("--images_topic", help="Name of a topic for images")
    parser.add_argument("--output_bag_file", help="Path to the output bag file")
    parser.add_argument('--times_from_filenames', action='store_true')
    parser.add_argument('--frames_to_skip', type=int, default=0)
    parser.add_argument('--downscale', action='store_true')
    args = parser.parse_args()

    convert(args)
