import numpy as np
import argparse

import rosbag, rospy
from nav_msgs.msg import Odometry


def convert(args):
    if not args.path_to_input:
        print("Please specify odom data .txt (.csv) file")
        return 1

    if not args.path_to_output:
        print("Please, specify output rosbag file")
        return 1

    odom_data = np.loadtxt(args.path_to_input, delimiter=" ")

    file_bag = args.path_to_output
    bag = rosbag.Bag(file_bag, 'w')

    times_usec = odom_data[:, 0]

    odom_xs = odom_data[:, 1]
    odom_ys = odom_data[:, 2]
    odom_zs = odom_data[:, 3]

    odom_qxs = odom_data[:, 4]
    odom_qys = odom_data[:, 5]
    odom_qzs = odom_data[:, 6]
    odom_qws = odom_data[:, 7]

    twist_linear_xs = odom_data[:, 8]
    twist_linear_ys = odom_data[:, 9]
    twist_linear_zs = odom_data[:, 10]

    twist_angular_xs = odom_data[:, 11]
    twist_angular_ys = odom_data[:, 12]
    twist_angular_zs = odom_data[:, 13]

    try:

        for i, time_usec in enumerate(times_usec):
            # TODO: make the factor config.
            TIMESTAMP_FACTOR = 1e9
            timestamp = rospy.Time.from_sec(time_usec / TIMESTAMP_FACTOR)

            odom_msg = Odometry()
            odom_msg.header.stamp = timestamp
            # TODO: make config.
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"

            odom_msg.pose.pose.position.x = odom_xs[i]
            odom_msg.pose.pose.position.y = odom_ys[i]
            odom_msg.pose.pose.position.z = odom_zs[i]

            odom_msg.pose.pose.orientation.x = odom_qxs[i]
            odom_msg.pose.pose.orientation.y = odom_qys[i]
            odom_msg.pose.pose.orientation.z = odom_qzs[i]
            odom_msg.pose.pose.orientation.w = odom_qws[i]

            odom_msg.twist.twist.linear.x = twist_linear_xs[i]
            odom_msg.twist.twist.linear.y = twist_linear_ys[i]
            odom_msg.twist.twist.linear.z = twist_linear_zs[i]

            odom_msg.twist.twist.angular.x = twist_angular_xs[i]
            odom_msg.twist.twist.angular.y = twist_angular_ys[i]
            odom_msg.twist.twist.angular.z = twist_angular_zs[i]

            bag.write(args.odom_topic, odom_msg, t=timestamp)

    finally:

        bag.close()

    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input', type=str, default='')
    parser.add_argument('--path_to_output', type=str, default='odom.bag')
    parser.add_argument('--odom_topic', type=str, default='/odom')
    args = parser.parse_args()

    convert(args)
