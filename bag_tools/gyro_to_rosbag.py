import numpy as np
import argparse

import rosbag, rospy
from sensor_msgs.msg import Imu


def convert(args):
    if not args.path_to_input:
        print("Please specify gyro data .txt (.csv) file")
        return 1

    if not args.path_to_output:
        print("Please, specify output rosbag file")
        return 1

    gyro_data = np.loadtxt(args.path_to_input, delimiter=",")

    file_bag = args.path_to_output
    bag = rosbag.Bag(file_bag, 'w')

    times_usec = gyro_data[:, 0]

    angular_velocity_r = gyro_data[:, 1]
    angular_velocity_p = gyro_data[:, 2]
    angular_velocity_y = gyro_data[:, 3]

    try:

        for i, time_usec in enumerate(times_usec):
            # TODO: make the factor config.
            timestamp_factor = 1
            timestamp = rospy.Time.from_sec(time_usec / timestamp_factor)

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            # TODO: make config.
            imu_msg.header.frame_id = "imu"

            imu_msg.angular_velocity.x = angular_velocity_r[i]
            imu_msg.angular_velocity.y = angular_velocity_p[i]
            imu_msg.angular_velocity.z = angular_velocity_y[i]

            bag.write(args.gyro_topic, imu_msg, t=timestamp)

    finally:

        bag.close()

    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input',
                        type=str,
                        default='',
                        help='Input file: time gx gy gz')
    parser.add_argument('--path_to_output',
                        type=str,
                        default='gyro.bag',
                        help='Output bag file')
    parser.add_argument('--gyro_topic',
                        type=str,
                        default='/gyro',
                        help='Gyro data topic')
    args = parser.parse_args()

    convert(args)
