import numpy as np
import argparse

import rosbag, rospy
from sensor_msgs.msg import Imu


class Interpolator:
    # TODO: configurate factors
    def __init__(self):
        self.start_time = None
        self.end_time = None
        self.times = None
        self.data = None
        self.interpolated_time_factor = 1  # ns -> s
        self.interpolated_time_difference = 0.01

    def prepare_times(self, input_file):
        self.data = input_file
        self.start_time = self.data[0, 0]
        self.end_time = self.data[-1, 0]
        self.end_time = int((self.end_time - self.start_time) / self.interpolated_time_factor)
        self.times = np.linspace(0, self.end_time, num=int(self.end_time / self.interpolated_time_difference))

    def get_interpolated_data(self):
        interpolated_data = np.zeros((self.times.shape[0], self.data.shape[1]))

        for i in range(0, self.data.shape[1]):
            interpolated_data[:, i] = np.interp(self.times,
                                                (self.data[:, 0] - self.start_time) / self.interpolated_time_factor,
                                                self.data[:, i])

        return interpolated_data


def convert(args):
    if not args.path_to_input:
        print("Please specify imu data .txt (.csv) file")
        return 1

    if not args.path_to_output:
        print("Please, specify output rosbag file")
        return 1

    imu_data = np.loadtxt(args.path_to_input, delimiter=" ")

    file_bag = args.path_to_output
    bag = rosbag.Bag(file_bag, 'w')

    if args.interpolate:
        interpolator = Interpolator()
        interpolator.prepare_times(imu_data)
        imu_data = interpolator.get_interpolated_data()

    if args.save_interpolated_data:
        np.savetxt(args.path_to_interpolated_data, imu_data, delimiter=",")

    times_usec = imu_data[:, 0]

    if args.reorder:
        acceleration_x = imu_data[:, 4]
        acceleration_y = imu_data[:, 5]
        acceleration_z = imu_data[:, 6]

        angular_velocity_r = imu_data[:, 1]
        angular_velocity_p = imu_data[:, 2]
        angular_velocity_y = imu_data[:, 3]
    else:
        acceleration_x = imu_data[:, 1]
        acceleration_y = imu_data[:, 2]
        acceleration_z = imu_data[:, 3]

        angular_velocity_r = imu_data[:, 4]
        angular_velocity_p = imu_data[:, 5]
        angular_velocity_y = imu_data[:, 6]

    try:

        for i, time_usec in enumerate(times_usec):
            # TODO: make the factor config.
            TIMESTAMP_FACTOR = 1e9
            timestamp = rospy.Time.from_sec(time_usec / TIMESTAMP_FACTOR)

            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            # TODO: make config.
            imu_msg.header.frame_id = "imu"

            imu_msg.linear_acceleration.x = acceleration_x[i]
            imu_msg.linear_acceleration.y = acceleration_y[i]
            imu_msg.linear_acceleration.z = acceleration_z[i]

            imu_msg.angular_velocity.x = angular_velocity_r[i]
            imu_msg.angular_velocity.y = angular_velocity_p[i]
            imu_msg.angular_velocity.z = angular_velocity_y[i]

            bag.write(args.imu_topic, imu_msg, t=timestamp)

    finally:

        bag.close()

    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input', type=str, default='',
                        help="Input file: time ax ay az gx gy gz")
    parser.add_argument('--path_to_output', type=str, default='imu.bag')
    parser.add_argument('--imu_topic', type=str, default='/imu')
    parser.add_argument('--reorder', action='store_true',
                        help="Use in case the order is as follows: gx, gy, gz, ax, ay, az")
    
    parser.add_argument('--interpolate', action='store_true',
                        help="Measurements will be interpolated using a frequency specified in Interpolator class")
    parser.add_argument('--save_interpolated_data', action='store_true')
    parser.add_argument('--path_to_interpolated_data', default='test_interpolation.csv')
    args = parser.parse_args()

    convert(args)
