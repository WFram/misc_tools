import numpy as np
import matplotlib.pyplot as plt
import argparse


def plot(args) -> None:
    imu_data = np.loadtxt(args.path_to_imu_data, delimiter=' ')

    x = range(len(imu_data[:, 1]))

    gyro_figure, (gyro_ax1, gyro_ax2, gyro_ax3) = plt.subplots(3, 1, figsize=(8, 8))
    gyro_figure.suptitle('Gyro measurements')

    gyro_ax1.plot(x, imu_data[:, 1], label=r'$\omega_x$')
    gyro_ax1.set_ylabel(r'$\omega_x$')
    gyro_ax1.grid()
    gyro_ax1.legend()

    gyro_ax2.plot(x, imu_data[:, 2], label=r'$\omega_y$')
    gyro_ax2.set_ylabel(r'$\omega_y$')
    gyro_ax2.grid()
    gyro_ax2.legend()

    gyro_ax3.plot(x, imu_data[:, 3], label=r'$\omega_z$')
    gyro_ax3.set_ylabel(r'$\omega_z$')
    gyro_ax3.grid()
    gyro_ax3.legend()

    acc_figure, (acc_ax1, acc_ax2, acc_ax3) = plt.subplots(3, 1, figsize=(8, 8))
    acc_figure.suptitle('Acc measurements')

    acc_ax1.plot(x, imu_data[:, 4], label=r'$a_x$')
    acc_ax1.set_ylabel(r'$a_x$')
    acc_ax1.grid()
    acc_ax1.legend()
    
    acc_ax2.plot(x, imu_data[:, 5], label=r'$a_y$')
    acc_ax2.set_ylabel(r'$a_y$')
    acc_ax2.grid()
    acc_ax2.legend()
    
    acc_ax3.plot(x, imu_data[:, 6], label=r'$a_z$')
    acc_ax3.set_ylabel(r'$a_z$')
    acc_ax3.grid()
    acc_ax3.legend()

    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--path_to_imu_data', type=str,
                        help="Input file: time gx gy gz ax ay az")
    args = parser.parse_args()

    plot(args)
