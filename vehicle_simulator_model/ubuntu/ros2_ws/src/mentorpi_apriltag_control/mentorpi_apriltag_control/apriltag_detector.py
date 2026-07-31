import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__("apriltag_detector")
        self.declare_parameter("image_topic", "/robot_1/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/robot_1/depth/camera_info")
        self.declare_parameter("target_topic", "/robot_1/apriltag/target")
        self.declare_parameter("tag_size", 0.20)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.params = cv2.aruco.DetectorParameters_create()

        self.publisher = self.create_publisher(
            String, self.get_parameter("target_topic").value, 10
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.on_camera_info,
            10,
        )
        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 10
        )
        self.get_logger().info("AprilTag detector started")

    def on_camera_info(self, msg):
        self.camera_matrix = np.array([
            [msg.k[0], msg.k[1], msg.k[2]],
            [msg.k[3], msg.k[4], msg.k[5]],
            [msg.k[6], msg.k[7], msg.k[8]],
        ], dtype=np.float64)
        self.dist_coeffs = np.array(
            msg.d if msg.d else [0.0, 0.0, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)

        detections = []
        if ids is not None:
            rvecs = tvecs = None
            if self.camera_matrix is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.tag_size,
                    self.camera_matrix,
                    self.dist_coeffs,
                )

            height, width = gray.shape
            for index, tag_id in enumerate(ids.flatten()):
                pts = corners[index].reshape((4, 2))
                center_x = float(pts[:, 0].mean())
                center_y = float(pts[:, 1].mean())
                side_lengths = [
                    float(np.linalg.norm(pts[(side + 1) % 4] - pts[side]))
                    for side in range(4)
                ]
                detection = {
                    "id": int(tag_id),
                    "center_x": center_x,
                    "center_y": center_y,
                    "tag_width_px": float(sum(side_lengths) / len(side_lengths)),
                    "error_x": (center_x - width / 2.0) / (width / 2.0),
                    "error_y": (center_y - height / 2.0) / (height / 2.0),
                    "image_width": int(width),
                    "image_height": int(height),
                }
                if tvecs is not None:
                    t = tvecs[index][0]
                    detection["translation"] = {
                        "x": float(t[0]),
                        "y": float(t[1]),
                        "z": float(t[2]),
                    }
                    detection["distance"] = float(t[2])
                detections.append(detection)

        payload = {
            "visible": bool(detections),
            "detections": detections,
        }
        self.publisher.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
