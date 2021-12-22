import rclpy
import cv2
import copy
import json
import os
import glob
import numpy as np
from ament_index_python.packages import get_package_share_directory

from typing import Dict, Tuple
from pathlib import Path
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class ImageRawPublisher(Node):
    def __init__(self):
        super().__init__("image_raw_publisher")
        # self.publisher_ = self.create_publisher(String, "image_raw", 10)
        self.image_raw_pub_ = self.create_publisher(Image, "image_raw", 10)
        self.camera_info_pub_ = self.create_publisher(CameraInfo, "camera_info", 10)
        self.frame_id = 0

        self.raw_publisher()
        # self.calibration()

    def raw_publisher(self) -> None:
        """
        Raw publisher routine (select this or calibration)
        """
        # set callback
        timer_period = 0.033  # seconds, 30 Hz approx
        # timer_period = 0.01  # seconds, 100 Hz approx
        # timer_period = 1.0  # seconds, 1 Hz approx
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # set reference default image_raw and camera_info data structures to publish
        package_share_path = get_package_share_directory("perception_2nodes")
        chessboard_path = os.path.join(package_share_path, "resource", "data")
        image_raw_path = os.path.join(chessboard_path, "image_raw.json")
        camera_info_path = os.path.join(chessboard_path, "camera_info.json")
        self.image_raw_ = self.image_from_json(Path(image_raw_path))
        self.camera_info_ = self.camera_info_from_json(Path(camera_info_path))

    def calibration(self) -> None:
        """
        Routine to calibrate

        Besides this routine, launch camera_calibration in a separate Node as:
        
            ros2 run camera_calibration cameracalibrator --size=7x6 \
                --square=0.1 --no-service-check --ros-args \
                --remap /image:=/image_raw
        """
        package_share_path = get_package_share_directory("perception_2nodes")
        chessboard_path = os.path.join(package_share_path, "resource", "data")

        self.counter_ = 0
        image_files = glob.glob(chessboard_path + "/opencv-*.jpg")
        self.images_ = []
        for image_path in image_files:
            self.get_logger().info(image_path)
            image_aux = self.image_from_path(image_path)
            self.images_.append(image_aux)

        timer_period = 1.0  # seconds, 1 Hz approx
        self.timer = self.create_timer(timer_period, self.timer_callback_calibrate)

    def image_from_path(self, image_path: String) -> Image:
        """
        Fetch image from its path.
        """
        # Load the main image data from a JSON-specified image file
        image = CvBridge().cv2_to_imgmsg(
            np.array(cv2.imread(image_path)),
            "bgr8",
        )
        image.encoding = "bgr8"
        return image

    def image_from_json(self, json_filepath: Path) -> Image:
        """
        Fetch image and dimensions from JSON.
        """
        with open(json_filepath) as json_file:
            image_json = json.load(json_file)

            # image_path_str = str(json_filepath.parent / image_json["image"])
            # image = cv2.imread(image_path_str)
            # image_bridge = CvBridge().cv2_to_imgmsg(
            #     np.array(image), image_json["encoding"]
            # )

            # Load the main image data from a JSON-specified image file
            image = CvBridge().cv2_to_imgmsg(
                np.array(cv2.imread(str(json_filepath.parent / image_json["image"]))),
                image_json["encoding"],
            )

            image.encoding = image_json["encoding"]
            return image

    def camera_info_from_json(self, json_filepath: Path) -> CameraInfo:
        """
        Fetch camera_info message from JSON.
        """
        with open(json_filepath) as json_file:
            camera_info_json = json.load(json_file)

            camera_info = CameraInfo()
            camera_info.header.frame_id = camera_info_json["header"]["frame_id"]
            camera_info.width = camera_info_json["width"]
            camera_info.height = camera_info_json["height"]
            camera_info.distortion_model = camera_info_json["distortion_model"]
            camera_info.d = camera_info_json["D"]
            camera_info.k = camera_info_json["K"]
            camera_info.r = camera_info_json["R"]
            camera_info.p = camera_info_json["P"]
            return camera_info

    def timer_callback_calibrate(self):
        self.get_logger().info("Publishing " + str(self.counter_))
        self.image_raw_pub_.publish(self.images_[self.counter_])
        self.counter_ += 1
        self.counter_ = self.counter_ % len(self.images_)

    def timer_callback(self):
        # image_raw = copy.copy(self.image_raw_)
        # camera_info = copy.copy(self.camera_info_)

        image_raw = self.image_raw_
        camera_info = self.camera_info_
        # self.frame_id += 1
        image_raw.header.frame_id = "tf_camera"

        # update timestamp in messages
        timestamp = self.get_clock().now().to_msg()
        image_raw.header.stamp = timestamp
        camera_info.header.stamp = timestamp

        # fix values manually for no transformation
        # # distortion coefficients
        # camera_info.d[0] = 0
        # camera_info.d[1] = 0
        # camera_info.d[2] = 0
        # camera_info.d[3] = 0
        # camera_info.d[4] = 0
        #
        # # camera matrix
        # camera_info.k[0] = 1
        # camera_info.k[1] = 0
        # camera_info.k[2] = camera_info.width / 2
        # camera_info.k[3] = 0
        # camera_info.k[4] = 1
        # camera_info.k[5] = camera_info.height / 2
        # camera_info.k[6] = 0
        # camera_info.k[7] = 0
        # camera_info.k[8] = 1
        #
        # # rotation vectors
        # camera_info.r[0] = 1
        # camera_info.r[1] = 0
        # camera_info.r[2] = 0
        # camera_info.r[3] = 0
        # camera_info.r[4] = 1
        # camera_info.r[5] = 0
        # camera_info.r[6] = 0
        # camera_info.r[7] = 0
        # camera_info.r[8] = 1
        #
        # # translation vectors
        # camera_info.p[0] = 1
        # camera_info.p[1] = 0
        # camera_info.p[2] = camera_info.width / 2
        # camera_info.p[3] = 0
        # camera_info.p[4] = 0
        # camera_info.p[5] = 1
        # camera_info.p[6] = camera_info.height / 2
        # camera_info.p[7] = 0
        # camera_info.p[8] = 0
        # camera_info.p[9] = 0
        # camera_info.p[10] = 1
        # camera_info.p[11] = 0

        # Publish test case over both topics
        self.image_raw_pub_.publish(image_raw)
        self.camera_info_pub_.publish(camera_info)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = ImageRawPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
