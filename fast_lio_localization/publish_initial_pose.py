#!/usr/bin/env python3

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
import tf_transformations


class PublishInitialPose(Node):
    def __init__(self):
        super().__init__("publish_initial_pose")
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def publish_pose(self, x, y, z, roll, pitch, yaw):
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        xyz = [x, y, z]

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = "map"
        self.pub_pose.publish(initial_pose)

        self.get_logger().info(f"Initial Pose: {x} {y} {z} {roll} {pitch} {yaw}")


def main(args=None):
    rclpy.init(args=args)
    node = PublishInitialPose()

    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("roll", type=float)
    parser.add_argument("pitch", type=float)
    parser.add_argument("yaw", type=float)
    args = parser.parse_args()

    node.publish_pose(args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
