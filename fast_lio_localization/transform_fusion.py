#!/usr/bin/env python3

import copy
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import rclpy.timer
import tf_transformations
import tf2_ros
from geometry_msgs.msg import Transform
from std_msgs.msg import Header


class TransformFusion(Node):
    def __init__(self):
        super().__init__("transform_fusion")

        self.cur_odom_to_baselink = None
        self.cur_map_to_odom = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_localization = self.create_publisher(Odometry, "/localization", 1)

        # Declare parameters for odom transformation
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish.use_odom_transform", True),
                ("publish.odom_roll", 180.0),
                ("publish.odom_pitch", -7.5),
                ("publish.odom_yaw", 0.0),
            ],
        )
        
        # Compute odom -> camera_init transformation matrix
        self.use_odom_transform = self.get_parameter("publish.use_odom_transform").value
        if self.use_odom_transform:
            roll = np.radians(self.get_parameter("publish.odom_roll").value)
            pitch = np.radians(self.get_parameter("publish.odom_pitch").value)
            yaw = np.radians(self.get_parameter("publish.odom_yaw").value)
            
            # Compute rotation matrix from RPY (ZYX convention)
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            
            self.T_odom_to_camera_init = np.eye(4)
            self.T_odom_to_camera_init[:3, :3] = np.array([
                [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                [-sp, cp*sr, cp*cr]
            ])
            
            self.get_logger().info(f"Odom transformation enabled: roll={self.get_parameter('publish.odom_roll').value}°, "
                                   f"pitch={self.get_parameter('publish.odom_pitch').value}°, "
                                   f"yaw={self.get_parameter('publish.odom_yaw').value}°")
        else:
            self.T_odom_to_camera_init = np.eye(4)

        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 1)
        self.create_subscription(Odometry, "/map_to_odom", self.cb_save_map_to_odom, 1)

        self.freq_pub_localization = 50
        self.timer = self.create_timer(1/self.freq_pub_localization, self.transform_fusion)
        # threading.Thread(target=self.transform_fusion, daemon=True).start()

    def pose_to_mat(self, pose_msg):
        trans = np.eye(4)
        trans[:3, 3] = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans

    def transform_fusion(self):
        if self.cur_odom_to_baselink is None:
            return

        # Use received T_map_to_odom, or default to identity if not yet received
        if self.cur_map_to_odom is not None:
            T_map_to_odom = self.pose_to_mat(self.cur_map_to_odom.pose.pose)
        else:
            # Will be at origin until 2D Pose Estimate is clicked
            T_map_to_odom = np.eye(4)

        # Publish TF chain depending on odom transform
        if self.use_odom_transform:
            # Publish: map -> odom
            transform_msg_odom = Transform()
            transform_msg_odom.translation.x = T_map_to_odom[0, 3]
            transform_msg_odom.translation.y = T_map_to_odom[1, 3]
            transform_msg_odom.translation.z = T_map_to_odom[2, 3]
            
            quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)
            transform_msg_odom.rotation.x = quat[0]
            transform_msg_odom.rotation.y = quat[1]
            transform_msg_odom.rotation.z = quat[2]
            transform_msg_odom.rotation.w = quat[3]
            
            transform_stamped_odom = tf2_ros.TransformStamped(
                header = self.cur_odom_to_baselink.header,
                child_frame_id = "odom",
                transform = transform_msg_odom
            )
            transform_stamped_odom.header.frame_id = "map"
            self.tf_broadcaster.sendTransform(transform_stamped_odom)
            
            # Publish: odom -> camera_init (static transform)
            transform_msg_camera = Transform()
            transform_msg_camera.translation.x = self.T_odom_to_camera_init[0, 3]
            transform_msg_camera.translation.y = self.T_odom_to_camera_init[1, 3]
            transform_msg_camera.translation.z = self.T_odom_to_camera_init[2, 3]
            
            quat_camera = tf_transformations.quaternion_from_matrix(self.T_odom_to_camera_init)
            transform_msg_camera.rotation.x = quat_camera[0]
            transform_msg_camera.rotation.y = quat_camera[1]
            transform_msg_camera.rotation.z = quat_camera[2]
            transform_msg_camera.rotation.w = quat_camera[3]
            
            transform_stamped_camera = tf2_ros.TransformStamped(
                header = self.cur_odom_to_baselink.header,
                child_frame_id = "camera_init",
                transform = transform_msg_camera
            )
            transform_stamped_camera.header.frame_id = "odom"
            self.tf_broadcaster.sendTransform(transform_stamped_camera)
        else:
            # Original behavior: map -> camera_init
            transform_msg = Transform()
            transform_msg.translation.x = T_map_to_odom[0, 3]
            transform_msg.translation.y = T_map_to_odom[1, 3]
            transform_msg.translation.z = T_map_to_odom[2, 3]
            
            quat = tf_transformations.quaternion_from_matrix(T_map_to_odom)
            transform_msg.rotation.x = quat[0]
            transform_msg.rotation.y = quat[1]
            transform_msg.rotation.z = quat[2]
            transform_msg.rotation.w = quat[3]
            
            transform_stamped_msg = tf2_ros.TransformStamped(
                    header = self.cur_odom_to_baselink.header,
                    child_frame_id = "camera_init",
                    transform = transform_msg
                )
            transform_stamped_msg.header.frame_id = "map"
            self.tf_broadcaster.sendTransform(transform_stamped_msg)

        cur_odom = copy.copy(self.cur_odom_to_baselink)
        if cur_odom is not None:
            # FAST-LIO publishes camera_init -> body
            T_camera_init_to_body = self.pose_to_mat(cur_odom.pose.pose)
            
            if self.use_odom_transform:
                # Compute: map -> odom -> camera_init -> body
                T_odom_to_body = np.matmul(self.T_odom_to_camera_init, T_camera_init_to_body)
                T_map_to_body = np.matmul(T_map_to_odom, T_odom_to_body)
            else:
                # Original: map -> camera_init -> body
                T_map_to_body = np.matmul(T_map_to_odom, T_camera_init_to_body)

            xyz = tf_transformations.translation_from_matrix(T_map_to_body)
            quat = tf_transformations.quaternion_from_matrix(T_map_to_body)

            localization = Odometry()
            localization.pose.pose = Pose(
                position = Point(x = xyz[0], y = xyz[1], z = xyz[2]), 
                orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])
            )
            localization.twist = cur_odom.twist

            localization.header.stamp = self.get_clock().now().to_msg()
            localization.header.frame_id = "map"
            localization.child_frame_id = "body"
            self.pub_localization.publish(localization)


    def cb_save_cur_odom(self, msg):
        self.cur_odom_to_baselink = msg

    def cb_save_map_to_odom(self, msg):
        self.cur_map_to_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = TransformFusion()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
