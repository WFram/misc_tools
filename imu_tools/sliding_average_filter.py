import numpy as np
import argparse
import matplotlib.pyplot as plt
import re


def transpose(t_a_b):
    t_b_a = np.transpose(t_a_b)
    return t_b_a

def invert(t_a_b):
    t_b_a = np.linalg.inv(t_a_b)
    return t_b_a


def plot_imu_data(raw_data, filtered_data):
    x = range(len(raw_data[:, 1]))

    gyro_figure, (gyro_ax1, gyro_ax2, gyro_ax3) = plt.subplots(3, 1, figsize=(8, 8))
    gyro_figure.suptitle('Gyro measurements')

    gyro_ax1.plot(x, raw_data[:, 1], label=r'$\omega_x$')
    gyro_ax1.plot(x, filtered_data[:, 1], label=r'$\omega_x$')
    gyro_ax1.set_ylabel(r'$\omega_x$')
    gyro_ax1.grid()
    gyro_ax1.legend()

    gyro_ax2.plot(x, raw_data[:, 2], label=r'$\omega_y$')
    gyro_ax2.plot(x, filtered_data[:, 2], label=r'$\omega_y$')
    gyro_ax2.set_ylabel(r'$\omega_y$')
    gyro_ax2.grid()
    gyro_ax2.legend()

    gyro_ax3.plot(x, raw_data[:, 3], label=r'$\omega_z$')
    gyro_ax3.plot(x, filtered_data[:, 3], label=r'$\omega_z$')
    gyro_ax3.set_ylabel(r'$\omega_z$')
    gyro_ax3.grid()
    gyro_ax3.legend()

    acc_figure, (acc_ax1, acc_ax2, acc_ax3) = plt.subplots(3, 1, figsize=(8, 8))
    acc_figure.suptitle('Acc measurements')

    acc_ax1.plot(x, raw_data[:, 4], label=r'$a_x$')
    acc_ax1.plot(x, filtered_data[:, 4], label=r'$a_x$')
    acc_ax1.set_ylabel(r'$a_x$')
    acc_ax1.grid()
    acc_ax1.legend()
    
    acc_ax2.plot(x, raw_data[:, 5], label=r'$a_y$')
    acc_ax2.plot(x, filtered_data[:, 5], label=r'$a_y$')
    acc_ax2.set_ylabel(r'$a_y$')
    # acc_ax2.set_ylim((-3, 3))
    acc_ax2.grid()
    acc_ax2.legend()
    
    acc_ax3.plot(x, raw_data[:, 6], label=r'$a_z$')
    acc_ax3.plot(x, filtered_data[:, 6], label=r'$a_z$')
    acc_ax3.set_ylabel(r'$a_z$')
    # acc_ax3.set_ylim((-3, 3))
    acc_ax3.grid()
    acc_ax3.legend()

    plt.show()


class SlidingAverageFilter:
    def __init__(self):
        self.window_size = 20
        self.id = 0

        self.measurement_window = np.identity(self.window_size + 1)


    def handle(self, raw_measurements: np.array((6, 1), dtype=np.float64)):
        for j in range(self.window_size):
            for i in range(6):
                if j == self.window_size - 1:
                    self.measurement_window[i, j] = raw_measurements[i]
                else:
                    self.measurement_window[i, j] = self.measurement_window[i, j + 1]

        filtered_measurements = np.zeros((6, 1), dtype=np.float64)
        for i in range(6):
            for j in range(self.window_size):
                filtered_measurements[i, 0] += self.measurement_window[i, j]

        return filtered_measurements / self.window_size


def run(args):
    if not args.path_to_input:
        print("Please specify imu data .txt (.csv) file")
        return 1

    if not args.path_to_output:
        print("Please, specify output rosbag file")
        return 1

    input_imu_data = np.loadtxt(args.path_to_input, delimiter=" ")
    output_imu_data = np.copy(input_imu_data)

    output_imu_file = open(args.path_to_output, "w")

    sliding_average_filter = SlidingAverageFilter()

    START = 0
    
    raw_measurements = np.zeros((6, 1), dtype=np.float64)

    for i in range(START, input_imu_data.shape[0]):
        for k, _ in enumerate(input_imu_data[i]):
            if k == 0:
                continue 
            raw_measurements[k - 1][0] = input_imu_data[i][k]

        if i - START < sliding_average_filter.window_size:
            for k, _ in enumerate(raw_measurements):
                sliding_average_filter.measurement_window[k, i - START] = raw_measurements[k][0]
            continue

        filtered_measurements = sliding_average_filter.handle(raw_measurements)
        
        for k, _ in enumerate(output_imu_data[i]):
            if k == 0:
                continue 
            output_imu_data[i][k] = filtered_measurements[k - 1][0]

        TIME_PRECISION = 0
        DATA_PRECISION = 14
        imu_id = "{:.{}f}".format(output_imu_data[i][0], TIME_PRECISION)
        imu_id = re.sub(r"\.", "", imu_id)
        
        output_imu_file.write(imu_id + " ")

        for k, _ in enumerate(output_imu_data[i]):
            if k == 0 or k == 6:
                continue
            output_imu_file.write("{}".format(output_imu_data[i][k], DATA_PRECISION) + " ")
        output_imu_file.write("{}".format(output_imu_data[i][6], DATA_PRECISION))
        output_imu_file.write("\n")

    if args.plot:
        plot_imu_data(input_imu_data, output_imu_data)

        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_input',
                        type=str,
                        required=True)
    parser.add_argument('--path_to_output',
                        type=str,
                        required=True)
    parser.add_argument('--plot',
                        action='store_true',
                        required=False)
    args = parser.parse_args()

    run(args)

