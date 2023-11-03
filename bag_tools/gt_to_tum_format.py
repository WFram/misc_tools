import rosbag
import argparse


def main():
    parser = argparse.ArgumentParser(description="Extract GT from a ROS bag")
    parser.add_argument("--bag_file", help="Input ROS bag")
    parser.add_argument("--output_file", help="Output GT txt file")
    parser.add_argument("--topic_name", help="Topic name")
    parser.add_argument("--message_type", help="Message type (geometry_msgs, nav_msgs)")
    args = parser.parse_args()

    bag_filename = args.bag_file
    output_filename = args.output_file
    topic_name = args.topic_name
    message_type = args.message_type

    gt_t = []
    gt = []

    with rosbag.Bag(bag_filename) as inbag:

        print(f"Processing: {bag_filename}")
        print(inbag)

        for topic, msg, t in inbag.read_messages():

            if topic == topic_name:
                gt_t.append(t)
                gt.append(msg)

    with open(output_filename, 'w') as outfile:
        outfile.write('#Time px py pz qx qy qz qw' + '\n')
        for msg, t in zip(gt, gt_t):

            if message_type == 'geometry_msgs':
                x = msg.transform.translation.x
                y = msg.transform.translation.y
                z = msg.transform.translation.z

                q_x = msg.transform.rotation.x
                q_y = msg.transform.rotation.y
                q_z = msg.transform.rotation.z
                q_w = msg.transform.rotation.w

            elif message_type == 'nav_msgs':
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = msg.pose.pose.position.z

                q_x = msg.pose.pose.orientation.x
                q_y = msg.pose.pose.orientation.y
                q_z = msg.pose.pose.orientation.z
                q_w = msg.pose.pose.orientation.w

            outfile.write(str(t.to_sec()) + ' ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(q_x) + ' ' + str(
                q_y) + ' ' + str(q_z) + ' ' + str(q_w) + '\n')

    outfile.close()


if __name__ == '__main__':
    main()
