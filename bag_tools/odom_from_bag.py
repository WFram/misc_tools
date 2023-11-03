import tqdm
import argparse

import rosbag, rospy


def extract(args):

    odom = open(args.path_to_output, "w")
    i = 0
    for topic, msg, t in tqdm.tqdm(rosbag.Bag(args.path_to_bag).read_messages()):

        if topic == args.topic_name:
            odom.write("{}".format(rospy.Time.to_sec(msg.header.stamp)) + " ")
            odom.write("{}".format(msg.pose.pose.position.x) + " ")
            odom.write("{}".format(msg.pose.pose.position.y) + " ")
            odom.write("{}".format(msg.pose.pose.position.z) + " ")
            odom.write("{}".format(msg.pose.pose.orientation.x) + " ")
            odom.write("{}".format(msg.pose.pose.orientation.y) + " ")
            odom.write("{}".format(msg.pose.pose.orientation.z) + " ")
            odom.write("{}".format(msg.pose.pose.orientation.w))
            odom.write("\n")

            i += 1


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_bag")
    parser.add_argument("--path_to_output")
    parser.add_argument("--topic_name")
    args = parser.parse_args()

    extract(args)
