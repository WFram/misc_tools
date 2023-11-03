#!/usr/bin/env python3

import argparse

import rosbag


def merge(args):
    bags = args.paths
    topics = args.topics
    output_bag = rosbag.Bag(args.output_bag_file, 'w')

    for bag in bags:
        with rosbag.Bag(bag) as input_bag:
            print(input_bag)
            for topic, msg, t in input_bag.read_messages():
                if topic in topics:
                    output_bag.write(topic, msg, t)
                

    output_bag.close()
    output_bag = rosbag.Bag(args.output_bag_file)
    print(output_bag)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output_bag_file',
                        type=str,
                        required=True)
    parser.add_argument('--paths',
                        nargs='+',
                        type=str,
                        required=True)
    parser.add_argument('--topics',
                        nargs='+',
                        type=str,
                        required=True)
    args = parser.parse_args()

    merge(args)
