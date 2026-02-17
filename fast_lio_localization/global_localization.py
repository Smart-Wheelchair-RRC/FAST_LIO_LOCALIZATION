#!/usr/bin/env python3

import copy
import threading
import time

import open3d as o3d
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
# from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf_transformations
import ros2_numpy
from rclpy.duration import Duration
from sensor_msgs_py import point_cloud2


class FastLIOLocalization(Node):
    def __init__(self):
        super().__init__("fast_lio_localization")
        self.global_map = None
        self.T_map_to_odom = np.eye(4)
        self.cur_odom = None
        self.cur_scan = None
        self.initialized = False

        self.declare_parameters(
            namespace="",
            parameters=[
                ("map_voxel_size", 0.4),
                ("scan_voxel_size", 0.1),
                ("freq_localization", 0.5),
                ("freq_global_map", 0.25),
                ("localization_threshold", 0.8),
                ("fov", 6.28319),
                ("fov_far", 300),
                ("pcd_map_topic", "/map"),
                ("pcd_map_path", ""),
                ("use_odom_transform", True),
                ("odom_roll", 180.0),
                ("odom_pitch", -7.5),
                ("odom_yaw", 0.0),
            ],
        )
        
        # Check if odom transformation is enabled
        self.use_odom_transform = self.get_parameter("use_odom_transform").value
        if self.use_odom_transform:
            self.get_logger().info(f"Odom transformation enabled: roll={self.get_parameter('odom_roll').value}°, "
                                   f"pitch={self.get_parameter('odom_pitch').value}°, "
                                   f"yaw={self.get_parameter('odom_yaw').value}°")
            self.get_logger().info("Will use TF tree to transform point clouds from camera_init to odom")

        # Initialize TF buffer and listener (must be before subscriptions)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # self.pub_global_map = self.create_publisher(PointCloud2, self.get_parameter("pcd_map_topic").value, 10)
        self.pub_pc_in_map = self.create_publisher(PointCloud2, "/cur_scan_in_map", 10)
        self.pub_submap = self.create_publisher(PointCloud2, "/submap", 10)
        self.pub_map_to_odom = self.create_publisher(Odometry, "/map_to_odom", 10)

        self.get_logger().info("Waiting for global map...")
        # global_map_msg = wait_for_message(msg_type = PointCloud2, node = self, topic = "/cloud_pcd")[1]
        # self.initialize_global_map(global_map_msg)
        
        self.initialize_global_map()
        self.get_logger().info("Global map received.")
        
        self.create_subscription(PointCloud2, "/cloud_registered", self.cb_save_cur_scan, 10)
        self.create_subscription(Odometry, "/Odometry", self.cb_save_cur_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.cb_initialize_pose, 10)

        self.timer_localisation = self.create_timer(1.0 / self.get_parameter("freq_localization").value, self.localisation_timer_callback)
        # self.timer_global_map = self.create_timer(1/ self.get_parameter("freq_global_map").value, self.global_map_callback)
        
        # Publish initial T_map_to_odom at origin immediately so transform_fusion can start
        self.publish_odom(self.T_map_to_odom)
        self.get_logger().info("Published initial map->odom transform at origin. Waiting for 2D Pose Estimate...")

    def global_map_callback(self):
        # self.get_logger().info(np.array(self.global_map.points).shape)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_global_map, header, np.array(self.global_map.points))
        
    def pose_to_mat(self, pose):
        trans = np.eye(4)
        trans[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def tf_to_matrix(self, transform):
        """Convert geometry_msgs/Transform to 4x4 transformation matrix"""
        trans = np.eye(4)
        trans[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
        quat = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        trans[:3, :3] = tf_transformations.quaternion_matrix(quat)[:3, :3]
        return trans
    
    def msg_to_array(self, pc_msg):
        pc_array = ros2_numpy.numpify(pc_msg)
        return pc_array["xyz"]
    
    def registration_at_scale(self, scan, map, initial, scale):
        result_icp = o3d.pipelines.registration.registration_icp(
        self.voxel_down_sample(scan, self.get_parameter("scan_voxel_size").value * scale),
        self.voxel_down_sample(map, self.get_parameter("map_voxel_size").value * scale),
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20),
        )
        return result_icp.transformation, result_icp.fitness
            
    def inverse_se3(self, trans):
        trans_inverse = np.eye(4)
        # R
        trans_inverse[:3, :3] = trans[:3, :3].T
        # t
        trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
        return trans_inverse



    def publish_point_cloud(self, publisher, header, pc):
        # Check valid input
        if pc is None or len(pc.shape) != 2 or pc.shape[0] == 0:
            return

        # Build fields
        fields = [
            point_cloud2.PointField(
                name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(
                name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(
                name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(
                name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1)
        ]

        # Ensure intensity exists
        if pc.shape[1] >= 4:
            intensity = pc[:, 3].astype(np.float32)
        else:
            intensity = np.zeros(pc.shape[0], dtype=np.float32)

        # Build Nx4 float32 array
        cloud = np.zeros((pc.shape[0], 4), dtype=np.float32)
        cloud[:, 0:3] = pc[:, 0:3]
        cloud[:, 3] = intensity

        # Convert to ROS2 message
        msg = point_cloud2.create_cloud(header, fields, cloud)
        publisher.publish(msg)

        
    def crop_global_map_in_FOV(self, pose_estimation):
        # FAST-LIO publishes odometry as camera_init -> body
        # cur_odom contains the transform from camera_init to body
        T_camera_init_to_body = self.pose_to_mat(self.cur_odom.pose.pose)
        
        # pose_estimation is T_map_to_odom (in odom frame, which is the correct orientation)
        if self.use_odom_transform:
            # Compute T_odom_to_camera_init from parameters
            roll = np.radians(self.get_parameter("odom_roll").value)
            pitch = np.radians(self.get_parameter("odom_pitch").value)
            yaw = np.radians(self.get_parameter("odom_yaw").value)
            
            # Create rotation matrix (ZYX convention)
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            
            T_odom_to_camera_init = np.eye(4)
            T_odom_to_camera_init[:3, :3] = np.array([
                [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                [-sp, cp*sr, cp*cr]
            ])
            
            # Compute body position in odom frame
            # T_odom_to_body = T_odom_to_camera_init * T_camera_init_to_body
            T_odom_to_body = np.matmul(T_odom_to_camera_init, T_camera_init_to_body)
            
            self.get_logger().debug('Using odom->camera_init->body transform chain for FOV cropping')
        else:
            # No odom transform: treat camera_init as odom
            T_odom_to_body = T_camera_init_to_body
            
        # Now compute body position in map frame
        # T_map_to_body = T_map_to_odom * T_odom_to_body
        T_map_to_body = np.matmul(pose_estimation, T_odom_to_body)
        T_body_to_map = self.inverse_se3(T_map_to_body)

        global_map_in_map = np.array(self.global_map.points)
        global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
        global_map_in_body = np.matmul(T_body_to_map, global_map_in_map.T).T

        if self.get_parameter("fov").value > 3.14:
            indices = np.where(
                (global_map_in_body[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_body[:, 1], global_map_in_body[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        else:
            indices = np.where(
                (global_map_in_body[:, 0] > 0)
                & (global_map_in_body[:, 0] < self.get_parameter("fov_far").value)
                & (np.abs(np.arctan2(global_map_in_body[:, 1], global_map_in_body[:, 0])) < self.get_parameter("fov").value / 2.0)
            )
        global_map_in_FOV = o3d.geometry.PointCloud()
        global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()   # ❗ use ROS time
        header.frame_id = "map"
        self.publish_point_cloud(self.pub_submap, header, np.array(global_map_in_FOV.points)[::10])

        return global_map_in_FOV

    def global_localization(self, pose_estimation):
        scan_tobe_mapped = copy.copy(self.cur_scan)
        global_map_in_FOV = self.crop_global_map_in_FOV(pose_estimation)
        
        # self.get_logger().info(f"Scan points: {len(scan_tobe_mapped.points)}, Map FOV points: {len(global_map_in_FOV.points)}")
        
        if len(scan_tobe_mapped.points) == 0:
            self.get_logger().error("Current scan is empty! Cannot perform localization.")
            return
        
        if len(global_map_in_FOV.points) == 0:
            self.get_logger().error("Map FOV is empty! Check initial pose or FOV parameters.")
            return
        
        transformation, _ = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)
        
        transformation, fitness = self.registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=1)
        
        # Log initial vs refined transform
        delta_x = transformation[0, 3] - pose_estimation[0, 3]
        delta_y = transformation[1, 3] - pose_estimation[1, 3]
        delta_z = transformation[2, 3] - pose_estimation[2, 3]
        delta_dist = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        
        self.get_logger().info(f"ICP fitness: {fitness:.4f} (threshold: {self.get_parameter('localization_threshold').value:.2f}), "
                              f"correction: {delta_dist:.3f}m [x:{delta_x:.3f}, y:{delta_y:.3f}, z:{delta_z:.3f}]")
        
        if fitness > self.get_parameter("localization_threshold").value:
            self.T_map_to_odom = transformation
            self.publish_odom(transformation)
            # self.get_logger().info("✓ Localization updated")
        else:
            self.get_logger().warn(f"✗ Localization rejected: low fitness score")

    def voxel_down_sample(self, pcd, voxel_size):
        # print(pcd)
        
        try:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        
        except Exception as e:
            # for opend3d 0.7 or lower
            pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
            
        return pcd_down

    def cb_save_cur_odom(self, msg):
        self.cur_odom = msg
        
    def cb_save_cur_scan(self, msg):
        # Do NOT rotate the point cloud manually.
        # Keep the scan in the original camera_init frame.
        pc = self.msg_to_array(msg)

        # Use the same header, including frame_id = 'camera_init'
        header = msg.header

        # Save the point cloud as-is (in camera_init frame)
        self.cur_scan = o3d.geometry.PointCloud()
        self.cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])

        # Publish for visualization (still in camera_init)
        self.publish_point_cloud(self.pub_pc_in_map, header, pc)

        
    def initialize_global_map(self): #, pc_msg):
        # self.global_map = o3d.geometry.PointCloud()
        # self.global_map.points = o3d.utility.Vector3dVector(self.msg_to_array(pc_msg)[:, :3])
        self.global_map = o3d.io.read_point_cloud(self.get_parameter("pcd_map_path").value)
        self.global_map = self.voxel_down_sample(self.global_map, self.get_parameter("map_voxel_size").value)
        # o3d.io.write_point_cloud("/home/wheelchair2/laksh_ws/pcds/lab_map_with_outside_corridor (with ground pcd)_downsampled.pcd", self.global_map)
        self.get_logger().info("Global map received.")

    def cb_initialize_pose(self, msg):
        # Initial pose from RViz is the robot's position in map frame
        T_map_to_body = self.pose_to_mat(msg.pose.pose)
        
        # We need to compute T_map_to_odom
        # T_map_to_odom = T_map_to_body * T_body_to_odom
        # where T_body_to_odom = inverse(T_odom_to_body)
        
        if self.cur_odom is not None and self.cur_scan is not None:
            # Get current odometry: camera_init -> body
            T_camera_init_to_body = self.pose_to_mat(self.cur_odom.pose.pose)
            
            if self.use_odom_transform:
                # Compute T_odom_to_camera_init from parameters (same as in transform_fusion.py)
                roll = np.radians(self.get_parameter("odom_roll").value)
                pitch = np.radians(self.get_parameter("odom_pitch").value)
                yaw = np.radians(self.get_parameter("odom_yaw").value)
                
                # Create rotation matrix (ZYX convention)
                cr, sr = np.cos(roll), np.sin(roll)
                cp, sp = np.cos(pitch), np.sin(pitch)
                cy, sy = np.cos(yaw), np.sin(yaw)
                
                T_odom_to_camera_init = np.eye(4)
                T_odom_to_camera_init[:3, :3] = np.array([
                    [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                    [-sp, cp*sr, cp*cr]
                ])
                
                # Compute T_odom_to_body = T_odom_to_camera_init * T_camera_init_to_body
                T_odom_to_body = np.matmul(T_odom_to_camera_init, T_camera_init_to_body)
                
                self.get_logger().info("Computed odom->camera_init->body transform chain")
            else:
                # No odom transform: camera_init acts as odom
                T_odom_to_body = T_camera_init_to_body
            
            # Compute T_map_to_odom = T_map_to_body * inverse(T_odom_to_body)
            T_body_to_odom = self.inverse_se3(T_odom_to_body)
            initial_T_map_to_odom = np.matmul(T_map_to_body, T_body_to_odom)
            
            self.get_logger().info(f"Initial pose received: x={T_map_to_body[0, 3]:.2f}, "
                                  f"y={T_map_to_body[1, 3]:.2f}, "
                                  f"z={T_map_to_body[2, 3]:.2f}")
            
            # Set initial T_map_to_odom and publish it immediately
            self.T_map_to_odom = initial_T_map_to_odom
            self.publish_odom(initial_T_map_to_odom)
 
            # Run scan matching to refine the estimate
            self.global_localization(initial_T_map_to_odom)
            self.initialized = True
        else:
            self.get_logger().warn("Cannot initialize: waiting for odometry and scan data")
            self.initialized = False
            
    def publish_odom(self, transform):

        # Extract translation
        x = transform[0, 3]
        y = transform[1, 3]

        # Extract yaw only (ignore roll & pitch)
        roll, pitch, yaw = tf_transformations.euler_from_matrix(transform)

        # Build clean 2D rotation
        R_clean = tf_transformations.euler_matrix(0.0, 0.0, yaw)

        qx, qy, qz, qw = tf_transformations.quaternion_from_matrix(R_clean)

        odom_msg = Odometry()
        odom_msg.pose.pose = Pose(
            position = Point(x=x, y=y, z=0.0),
            orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        )

        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"

        self.pub_map_to_odom.publish(odom_msg)


    def localisation_timer_callback(self):
        if not self.initialized:
            self.get_logger().info("Waiting for initial pose...")
            return
        
        if self.cur_scan is not None:
            self.global_localization(self.T_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FastLIOLocalization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()