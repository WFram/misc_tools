import argparse
import os
import sys
import tqdm
import re

import cv2

import rosbag
import rospy
from cv_bridge import CvBridge


def downscale_image(input_image):
    scale_percent = 50

    width = int(input_image.shape[1] * scale_percent / 100)
    height = int(input_image.shape[0] * scale_percent / 100)
    dim = (width, height)

    output_image = cv2.resize(input_image, dim, interpolation=cv2.INTER_AREA)

    return output_image


def extract_images(options):
    input_bag = options.path_to_bag
    camera_topic = options.topic
    output_folder = options.path_to_output
    downscale = options.downscale
    generate_times = options.generate_times

    if not os.path.exists(output_folder + "/images/"):
        os.makedirs(output_folder + "/images/")
    output_folder = output_folder + "/images/"
    times = open(os.path.join(output_folder, "../times.txt"), "w")

    TIME_PRECISION = 4
    bridge = CvBridge()
    for topic, msg, _ in tqdm.tqdm(rosbag.Bag(input_bag).read_messages()):
        if topic == camera_topic:
            image_id = "{:.{}f}".format(rospy.Time.to_sec(msg.header.stamp), TIME_PRECISION)
            image_id = re.sub(r"\.", "", image_id)
            if generate_times:
                times.write(image_id + " ")
                times.write("{}".format(rospy.Time.to_sec(msg.header.stamp)) + " ")
                times.write("\n")

            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            if downscale:
                img = downscale_image(img)
            
            cv2.imwrite(os.path.join(output_folder, "{}.png".format(image_id)), img)

    times.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images from a bag file")
    parser.add_argument('--path_to_bag', type=str, default='')
    parser.add_argument('--path_to_output', type=str, default='images/')
    parser.add_argument('--topic', type=str, default='/camera/image_raw')
    parser.add_argument('--downscale', action='store_true')
    parser.add_argument('--generate_times', action='store_true')
    args = parser.parse_args()

    sys.exit(extract_images(args))
