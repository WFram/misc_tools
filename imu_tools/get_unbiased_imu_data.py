#!/usr/bin/env python

import os
import numpy as np
import argparse


def get_unbiased_data(args):
    path_to_input = args.path_to_input_file
    path_to_bias = args.path_to_bias_file
    path_to_output = args.path_to_output_file

    biased_imu_data = np.loadtxt(path_to_input, delimiter=' ')

    bias = np.loadtxt(path_to_bias, delimiter=' ')

    biased_imu_data[:, 1:4] -= bias[:3]
    biased_imu_data[:, 4:] -= bias[3:]

    unbiased_imu_data = np.zeros([biased_imu_data.shape[0], biased_imu_data.shape[1]], dtype=float)
    unbiased_imu_data[:, 0] = biased_imu_data[:, 0]
    unbiased_imu_data[:, 1:4] = biased_imu_data[:, 1:4] - bias[:3]
    unbiased_imu_data[:, 4:] = biased_imu_data[:, 4:] - bias[3:]

    output = open(path_to_output, "w")
    for row in range(unbiased_imu_data.shape[0]):
        output.write("{}".format(unbiased_imu_data[row, 0]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 1]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 2]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 3]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 4]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 5]) + " ")
        output.write("{}".format(unbiased_imu_data[row, 6]))
        output.write("\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input_file', type=str)
    parser.add_argument('--path_to_bias_file', type=str)
    parser.add_argument('--path_to_output_file', type=str)
    args = parser.parse_args()

    get_unbiased_data(args)
