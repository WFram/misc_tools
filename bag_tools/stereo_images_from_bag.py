import argparse
import os
import sys
import tqdm

import cv2

import rosbag
import rospy
from cv_bridge import CvBridge


def extract_images(options):
    input_bag = options.path_to_bag
    left_camera_topic = options.left_image_topic
    right_camera_topic = options.right_image_topic
    output_folder = options.path_to_output
    generate_times = options.generate_times

    os.makedirs(output_folder + "/images_left")
    os.makedirs(output_folder + "/images_right")

    left_output_folder = output_folder + "/images_left"
    right_output_folder = output_folder + "/images_right"

    left_times = open(os.path.join(left_output_folder, "../left.txt"), "w")
    right_times = open(os.path.join(right_output_folder, "../right.txt"), "w")

    bridge = CvBridge()
    for topic, msg, t in tqdm.tqdm(rosbag.Bag(input_bag).read_messages()):
        if topic == left_camera_topic:
            if generate_times:
                left_times.write("{}".format(rospy.Time.to_sec(msg.header.stamp)) + " ")
                left_times.write("./{}.png".format(rospy.Time.to_sec(msg.header.stamp)))
                left_times.write("\n")

            left_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(left_output_folder, "{}.png".format(rospy.Time.to_sec(msg.header.stamp))),
                        left_img)

        if topic == right_camera_topic:
            if generate_times:
                right_times.write("{}".format(rospy.Time.to_sec(msg.header.stamp)) + " ")
                right_times.write("./{}.png".format(rospy.Time.to_sec(msg.header.stamp)))
                right_times.write("\n")

            right_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            cv2.imwrite(os.path.join(right_output_folder, "{}.png".format(rospy.Time.to_sec(msg.header.stamp))),
                        right_image)

    left_times.close()
    right_times.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract images from a bag file")
    parser.add_argument('--path_to_bag', type=str, default='')
    parser.add_argument('--path_to_output', type=str, default='images/')
    parser.add_argument('--left_image_topic', type=str, default='/camera/left/image_raw')
    parser.add_argument('--right_image_topic', type=str, default='/camera/right/image_raw')
    parser.add_argument('--generate_times', action='store_true')
    args = parser.parse_args()

    sys.exit(extract_images(args))
