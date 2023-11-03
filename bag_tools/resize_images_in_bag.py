import argparse

import cv2 as cv

import rosbag
from cv_bridge import CvBridge


def downscale_image(input_image, resize_ratio):
    width = int(input_image.shape[1] * resize_ratio)
    height = int(input_image.shape[0] * resize_ratio)
    dim = (width, height)

    output_image = cv.resize(input_image, dim, interpolation=cv.INTER_AREA)

    return output_image


def run(args) -> None:
    input_bag_file = args.input_bag_file
    output_bag_file = args.output_bag_file
    resize_ratio = args.resize_ratio
    topics = args.topics

    output_bag = rosbag.Bag(output_bag_file, 'w')
    bridge = CvBridge()

    with rosbag.Bag(input_bag_file) as input_bag:
            print(input_bag)
            for topic, input_msg, t in input_bag.read_messages():
                if topic in topics:
                    input_image = bridge.imgmsg_to_cv2(input_msg, desired_encoding="bgr8")
                    output_image = downscale_image(input_image, resize_ratio)
                    output_msg = bridge.cv2_to_imgmsg(output_image, encoding="rgb8")
                    output_msg.header.stamp = input_msg.header.stamp
                    output_msg.header.frame_id = input_msg.header.frame_id
                    output_bag.write(topic, output_msg, t)
                else:
                    output_bag.write(topic, input_msg, t)
                

    output_bag.close()
    output_bag = rosbag.Bag(output_bag_file)
    print(output_bag)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_bag_file',
                        type=str,
                        required=True)
    parser.add_argument('--output_bag_file',
                        type=str,
                        required=True)
    parser.add_argument('--resize_ratio',
                        type=float,
                        required=True)
    parser.add_argument('--topics',
                        nargs='+',
                        type=str,
                        required=True)
    args = parser.parse_args()

    run(args)