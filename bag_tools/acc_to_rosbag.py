import numpy as np
import argparse

import rosbag, rospy
from sensor_msgs.msg import Imu


def convert(args):
    if not args.path_to_input:
        print("Please specify accelerometer data .txt (.csv) file")
        return 1

    if not args.path_to_output:
        print("Please, specify output rosbag file")
        return 1

    acc_data = np.loadtxt(args.path_to_input, delimiter=" ")

    file_bag = args.path_to_output
    bag = rosbag.Bag(file_bag, 'w')

    times_usec = acc_data[:, 0]

    acceleration_x = acc_data[:, 1]
    acceleration_y = acc_data[:, 2]
    acceleration_z = acc_data[:, 3]

    try:

        for i, time_usec in enumerate(times_usec):
            # TODO: make the factor configurable
            TIMESTAMP_FACTOR = 1
            timestamp = rospy.Time.from_sec(time_usec / TIMESTAMP_FACTOR)

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            # TODO: make configurable
            imu_msg.header.frame_id = "imu"

            imu_msg.linear_acceleration.x = acceleration_x[i]
            imu_msg.linear_acceleration.y = acceleration_y[i]
            imu_msg.linear_acceleration.z = acceleration_z[i]

            bag.write(args.acc_topic, imu_msg, t=timestamp)

    finally:

        bag.close()

    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input',
                        type=str,
                        default='',
                        help='Input file. time ax ay az')
    parser.add_argument('--path_to_output',
                        type=str,
                        default='acc.bag',
                        help='Output bag file')
    parser.add_argument('--acc_topic',
                        type=str,
                        default='/acc',
                        help='Accelerometer data topic')
    args = parser.parse_args()

    convert(args)
