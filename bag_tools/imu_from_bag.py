import rosbag, rospy

import tqdm
import argparse
import re


def extract(args):
    imu = open(args.path_to_output, "w")
    TIME_PRECISION = 4
    for topic, msg, _ in tqdm.tqdm(rosbag.Bag(args.path_to_bag).read_messages()):

        if topic == args.topic_name:
            imu_id = "{:.{}f}".format(rospy.Time.to_sec(msg.header.stamp), TIME_PRECISION)
            imu_id = re.sub(r"\.", "", imu_id)
            
            imu.write(imu_id + " ")
            imu.write("{}".format(msg.angular_velocity.x) + " ")
            imu.write("{}".format(msg.angular_velocity.y) + " ")
            imu.write("{}".format(msg.angular_velocity.z) + " ")
            imu.write("{}".format(msg.linear_acceleration.x) + " ")
            imu.write("{}".format(msg.linear_acceleration.y) + " ")
            imu.write("{}".format(msg.linear_acceleration.z))
            imu.write("\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_bag")
    parser.add_argument("--path_to_output")
    parser.add_argument("--topic_name")
    args = parser.parse_args()

    extract(args)
