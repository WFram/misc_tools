import argparse
import os
import numpy as np
import cv2 as cv

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# TODO: set from .txt file
K1 = 0.0005
K2 = -0.0001
P1 = 8.54e-05
P2 = 0.001

FX = 529.888
FY = 532.263
CX = 643.713
CY = 358.804


def read_image_file(path):
    return cv.imread(path)


def se3_to_buffer(se3_pose):
    pose_buffer = se3_pose.reshape((1, 16))
    pose_buffer = np.squeeze(pose_buffer)
    pose_buffer = ' '.join(str(i) for i in pose_buffer)

    return pose_buffer


class ArucoDetector:
    def __init__(self, args) -> None:
        if not args.test_mode:
            rospy.init_node('aruco_detector_node')
            self.bridge = CvBridge()
            self.raw_image_topic = args.image_topic
            self.raw_image_subscriber = rospy.Subscriber(self.raw_image_topic, Image, self.image_callback)

        self.poses_file = args.path_to_poses
        
        self.aruco_dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
        self.parameters = cv.aruco.DetectorParameters_create()

        self.intrinsic_matrix = np.array([[FX, 0., CX],
                                          [0., FY, CY],
                                          [0., 0., 1.]])
        self.distortion_coefficients = np.array([[K1, K2, P1, P2]])

    def detect(self, image) -> None:
        corners, indices, _ = cv.aruco.detectMarkers(image, self.aruco_dictionary, parameters=self.parameters)

        if indices is None:
            print(f"Lost ARUCO")
            return

        r_vectors, t_vectors, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_matrix,
                                                                     self.distortion_coefficients)

        r_camera_aruco, _ = cv.Rodrigues(r_vectors)
        t_camera_aruco = np.eye(4)
        t_camera_aruco[:3, :3] = r_camera_aruco
        t_camera_aruco[:3, 3] = t_vectors.squeeze()

        print(f"Resulting pose: \n")
        print(f"{t_camera_aruco}")

        if self.poses_file:
            pose_buffer = se3_to_buffer(t_camera_aruco)
            with open(self.poses_file, 'a') as output_file:
                output_file.write(f'{pose_buffer}\n')

    def image_callback(self, msg) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        self.detect(cv_image)


def main() -> None:
    parser = argparse.ArgumentParser(description="ARUCO detector ROS node. Can be used on a sample image")
    parser.add_argument('--test_mode',
                        action='store_true',
                        help='Not a ROS node',
                        required=False)
    parser.add_argument('--image_topic',
                        type=str,
                        required=False)
    parser.add_argument('--path_to_image',
                        type=str,
                        required=False)
    parser.add_argument('--path_to_poses',
                        type=str,
                        required=False,
                        help='Path to output ARUCO poses')
    parser.add_argument('--show_image',
                        action='store_true',
                        required=False)
    args = parser.parse_args()

    aruco_detector = ArucoDetector(args)

    if args.test_mode:
        image = read_image_file(args.path_to_image)
        aruco_detector.detect(image)
        if args.show_image:
            cv.imshow('image', image)
            cv.waitKey(0)

    if not args.test_mode:
        while not rospy.is_shutdown():
            rospy.spin()


if __name__ == '__main__':
    main()
