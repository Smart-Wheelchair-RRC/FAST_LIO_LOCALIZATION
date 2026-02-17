#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import numpy as np
import tf_transformations


class FastLioOdomConverter(Node):
    def __init__(self):
        super().__init__("fastlio_odom_converter")

        # Declare ROS2 parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish.use_odom_transform", False),
                ("publish.odom_roll", 0.0),
                ("publish.odom_pitch", 0.0),
                ("publish.odom_yaw", 0.0),
            ],
        )

        # Check if odom transformation is enabled
        self.use_odom_transform = self.get_parameter("publish.use_odom_transform").value
        
        if not self.use_odom_transform:
            self.get_logger().info("Odom transform disabled - node will not process odometry")
            # Still create subscriber/publisher but won't publish
            self.sub = self.create_subscription(Odometry, "/Odometry", self.cb, 10)
            self.pub = self.create_publisher(Odometry, "/odom", 10)
            return

        # --- STATIC TRANSFORMS ---------------------------------------
        # 1. odom -> camera_init
        odom_roll = np.radians(self.get_parameter("publish.odom_roll").value)
        odom_pitch = np.radians(self.get_parameter("publish.odom_pitch").value)
        odom_yaw = np.radians(self.get_parameter("publish.odom_yaw").value)
        R1 = self.rpy_to_matrix(odom_roll, odom_pitch, odom_yaw)

        # 2. body -> base_link (inverse of odom -> camera_init)
        # Since odom has the same orientation as base_link, and camera_init has the same orientation as body,
        # body -> base_link is simply the inverse of odom -> camera_init
        R2 = np.linalg.inv(R1)

        # Combined static transform:
        # odom -> base_link = (odom->camera_init) * (body->base_link)
        self.T_static = R1 @ R2

        # Subscribers and publishers
        self.sub = self.create_subscription(Odometry, "/Odometry", self.cb, 10)
        self.pub = self.create_publisher(Odometry, "/odom", 10)

        self.get_logger().info(f"FAST-LIO odom converter started with parameters:")
        self.get_logger().info(f"  odom->camera_init: roll={self.get_parameter('publish.odom_roll').value}°, "
                              f"pitch={self.get_parameter('publish.odom_pitch').value}°, "
                              f"yaw={self.get_parameter('publish.odom_yaw').value}°")
        self.get_logger().info(f"  body->base_link: computed as inverse (to flip back)")

    # Utility: Convert RPY to 4x4 matrix
    def rpy_to_matrix(self, roll, pitch, yaw):
        T = tf_transformations.euler_matrix(roll, pitch, yaw)
        return T

    # Utility: FAST-LIO odometry: camera_init -> body -> matrix
    def pose_to_mat(self, pose):
        T = np.eye(4)
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        T[:3, :3] = tf_transformations.quaternion_matrix(q)[:3, :3]
        return T

    def cb(self, msg):
        # If odom transform is disabled, do nothing
        if not self.use_odom_transform:
            return
            
        # camera_init -> body (dynamic odometry)
        T_cam_to_body = self.pose_to_mat(msg.pose.pose)

        # odom -> base_link = static ⨉ dynamic
        T_odom_to_base = self.T_static @ T_cam_to_body

        # Extract xyz and quaternion
        xyz = T_odom_to_base[:3, 3]
        quat = tf_transformations.quaternion_from_matrix(T_odom_to_base)

        # Construct new odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp    # OK to reuse
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position = Point(
            x=xyz[0], y=xyz[1], z=xyz[2]
        )
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom.twist = msg.twist  # same twist

        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLioOdomConverter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()