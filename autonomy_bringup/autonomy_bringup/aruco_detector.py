#!/usr/bin/env python3
"""
OpenCV-based ArUco detector for the two-robot Gazebo scenario.

Subscribes to a camera stream, estimates marker poses, publishes TF frames and
RViz markers, and emits a debug image with detected corners and axes.
"""

from __future__ import annotations

import math
from typing import Iterable

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def _rotation_matrix_to_quaternion(rot: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an (x, y, z, w) quaternion."""
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rot[2, 1] - rot[1, 2]) * s
        y = (rot[0, 2] - rot[2, 0]) * s
        z = (rot[1, 0] - rot[0, 1]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2])
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2])
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1])
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _load_dictionary(dictionary_name: str):
    dict_id = getattr(cv2.aruco, dictionary_name)
    if hasattr(cv2.aruco, 'getPredefinedDictionary'):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def _make_detector(aruco_dict):
    if hasattr(cv2.aruco, 'DetectorParameters'):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, 'ArucoDetector'):
        return cv2.aruco.ArucoDetector(aruco_dict, params), None
    return None, params


def _normalize_target_ids(raw_value) -> set[int]:
    if raw_value in (None, '', []):
        return set()
    if isinstance(raw_value, str):
        cleaned = raw_value.strip().strip('[]')
        if not cleaned:
            return set()
        return {int(part.strip()) for part in cleaned.split(',') if part.strip()}
    return {int(v) for v in raw_value}


class ArucoDetector(Node):
    def __init__(self) -> None:
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/a300_00000/sensors/camera_0/color/image')
        self.declare_parameter(
            'camera_info_topic', '/a300_00000/sensors/camera_0/color/camera_info')
        self.declare_parameter('marker_length_m', 0.4)
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('target_ids', [])
        self.declare_parameter('frame_prefix', 'aruco_marker')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('use_sim_time', True)

        self._bridge = CvBridge()
        self._camera_matrix: np.ndarray | None = None
        self._dist_coeffs: np.ndarray | None = None
        self._camera_frame: str | None = None
        self._waiting_for_info_logged = False
        self._marker_length = float(self.get_parameter('marker_length_m').value)
        self._frame_prefix = str(self.get_parameter('frame_prefix').value)
        self._publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self._target_ids = _normalize_target_ids(self.get_parameter('target_ids').value)
        self._aruco_dict = _load_dictionary(str(self.get_parameter('dictionary').value))
        self._detector, self._detector_params = _make_detector(self._aruco_dict)

        image_topic = str(self.get_parameter('image_topic').value)
        camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self._pose_pub = self.create_publisher(PoseArray, '~/poses', 10)
        self._marker_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self.get_logger().info(
            f'Aruco detector listening on image={image_topic} camera_info={camera_info_topic}')

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if not any(msg.k):
            return
        self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self._dist_coeffs = np.array(msg.d, dtype=np.float64)
        self._camera_frame = msg.header.frame_id or self._camera_frame or 'camera'
        self._waiting_for_info_logged = False

    def _detect_markers(self, gray_image: np.ndarray):
        if self._detector is not None:
            return self._detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(
            gray_image,
            self._aruco_dict,
            parameters=self._detector_params,
        )

    def _publish_empty(self, header, debug_frame: np.ndarray | None = None) -> None:
        poses = PoseArray()
        poses.header = header
        self._pose_pub.publish(poses)

        delete_all = Marker()
        delete_all.header = header
        delete_all.action = Marker.DELETEALL
        self._marker_pub.publish(MarkerArray(markers=[delete_all]))

        if self._publish_debug_image and debug_frame is not None:
            debug_msg = self._bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            debug_msg.header = header
            self._debug_pub.publish(debug_msg)

    def _image_cb(self, msg: Image) -> None:
        if self._camera_matrix is None or self._dist_coeffs is None:
            if not self._waiting_for_info_logged:
                self.get_logger().info('Waiting for valid camera_info before ArUco detection starts')
                self._waiting_for_info_logged = True
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        header = msg.header
        header.frame_id = self._camera_frame or msg.header.frame_id

        if ids is None or len(ids) == 0:
            self._publish_empty(header, frame)
            return

        ids = ids.flatten()
        if self._target_ids:
            keep = [idx for idx, marker_id in enumerate(ids) if int(marker_id) in self._target_ids]
            corners = [corners[idx] for idx in keep]
            ids = ids[keep]
            if len(ids) == 0:
                self._publish_empty(header, frame)
                return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_length, self._camera_matrix, self._dist_coeffs)

        pose_array = PoseArray()
        pose_array.header = header

        markers = [Marker(header=header, action=Marker.DELETEALL)]
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for index, marker_id in enumerate(ids):
            rvec = rvecs[index].reshape(3)
            tvec = tvecs[index].reshape(3)
            rot_matrix, _ = cv2.Rodrigues(rvec)
            quat = _rotation_matrix_to_quaternion(rot_matrix)

            pose = Pose()
            pose.position.x = float(tvec[0])
            pose.position.y = float(tvec[1])
            pose.position.z = float(tvec[2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            pose_array.poses.append(pose)

            transform = TransformStamped()
            transform.header = header
            transform.child_frame_id = f'{self._frame_prefix}_{int(marker_id)}'
            transform.transform.translation.x = pose.position.x
            transform.transform.translation.y = pose.position.y
            transform.transform.translation.z = pose.position.z
            transform.transform.rotation.x = pose.orientation.x
            transform.transform.rotation.y = pose.orientation.y
            transform.transform.rotation.z = pose.orientation.z
            transform.transform.rotation.w = pose.orientation.w
            self._tf_broadcaster.sendTransform(transform)

            marker = Marker()
            marker.header = header
            marker.ns = 'aruco_detections'
            marker.id = int(marker_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pose.position.x
            marker.pose.position.y = pose.position.y
            marker.pose.position.z = pose.position.z
            marker.pose.orientation.x = pose.orientation.x
            marker.pose.orientation.y = pose.orientation.y
            marker.pose.orientation.z = pose.orientation.z
            marker.pose.orientation.w = pose.orientation.w
            marker.scale.x = 0.01
            marker.scale.y = self._marker_length
            marker.scale.z = self._marker_length
            marker.color.r = 0.1
            marker.color.g = 0.9
            marker.color.b = 0.2
            marker.color.a = 0.55
            markers.append(marker)

            if hasattr(cv2.aruco, 'drawAxis'):
                cv2.aruco.drawAxis(
                    frame,
                    self._camera_matrix,
                    self._dist_coeffs,
                    rvec,
                    tvec,
                    self._marker_length * 0.5,
                )

        self._pose_pub.publish(pose_array)
        self._marker_pub.publish(MarkerArray(markers=markers))

        if self._publish_debug_image:
            debug_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self._debug_pub.publish(debug_msg)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
