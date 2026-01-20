#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomComparator(Node):
    def __init__(self):
        super().__init__('odom_comparator')

        self.fastlio_odom = None   # /Odometry
        self.converted_odom = None # /odom

        self.sub_fastlio = self.create_subscription(
            Odometry, '/Odometry', self.cb_fastlio, 10)

        self.sub_converted = self.create_subscription(
            Odometry, '/odom', self.cb_converted, 10)

        self.get_logger().info("Comparing /Odometry and /odom positions...")

    def cb_fastlio(self, msg):
        self.fastlio_odom = msg
        self.compare()

    def cb_converted(self, msg):
        self.converted_odom = msg
        self.compare()

    def compare(self):
        if self.fastlio_odom is None or self.converted_odom is None:
            return  # not enough data yet

        p1 = self.fastlio_odom.pose.pose.position
        p2 = self.converted_odom.pose.pose.position

        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z

        # Print both positions and differences
        self.get_logger().info(
            f"\nFAST-LIO /Odometry position: "
            f"({p1.x:.4f}, {p1.y:.4f}, {p1.z:.4f})\n"
            f"Converted /odom position:     "
            f"({p2.x:.4f}, {p2.y:.4f}, {p2.z:.4f})\n"
            f"Difference (dx, dy, dz):      "
            f"({dx:.6f}, {dy:.6f}, {dz:.6f})"
        )

        # Optional: print warning if difference > threshold
        tol = 1e-3  # 1 mm tolerance
        if abs(dx) > tol or abs(dy) > tol or abs(dz) > tol:
            self.get_logger().warn("⛔ Positions DO NOT MATCH within tolerance!")
        else:
            self.get_logger().info("✅ Positions MATCH (as expected).")


def main(args=None):
    rclpy.init(args=args)
    node = OdomComparator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
