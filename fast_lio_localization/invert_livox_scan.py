#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.parameter
import rclpy.parameter_service
from sensor_msgs.msg import PointCloud2, Imu
from livox_ros_driver2.msg import CustomMsg
import ros2_numpy
# from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Ensure reliable message delivery
    history=HistoryPolicy.KEEP_LAST,        # Keep the last few messages
    depth=10                                # Increase buffer size
)

class LivoxLaserToPointcloud(Node):
    def __init__(self):
        super().__init__("Invert_Livox_Scan")

        xfer_format = self.declare_parameter("xfer_format", 0).value

        if xfer_format == 0:
            self.pub_scan = self.create_publisher(PointCloud2, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(PointCloud2, "/livox/inverted_lidar", self.pointcloud2_callback, qos_profile=qos_profile)

        elif xfer_format == 1:
            self.pub_scan = self.create_publisher(CustomMsg, "/livox/lidar", qos_profile=qos_profile)
            self.sub_scan = self.create_subscription(CustomMsg, "/livox/inverted_lidar", self.custom_msg_callback, qos_profile=qos_profile)

        else:
            self.get_logger().error(f"Method undefined for xfer_format = {xfer_format}")
            self.destroy_node()
            
            return

        self.pub_imu = self.create_publisher(Imu, "/livox/imu", qos_profile=qos_profile)
        self.sub_imu = self.create_subscription(Imu, "/livox/inverted_imu", self.imu_callback, qos_profile=qos_profile)
        

    def pointcloud2_callback(self, msg: PointCloud2):
        data = ros2_numpy.numpify(msg)
        # print(data)
        
        pc = data['xyz']
        # print(pc)
        pc[:, 1] = -pc[:, 1]
        pc[:, 2] = -pc[:, 2]
        # print(pc)
        
        data = {"xyz": pc}  # Invert Y, Z
        # print(data)

        out_msg = ros2_numpy.msgify(PointCloud2, data)
        out_msg.header = msg.header
        # out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.point_step = 12
        self.pub_scan.publish(out_msg)

    def custom_msg_callback(self, msg: CustomMsg):
        for p in msg.points:
            p.y = -p.y
            p.z = -p.z
            
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.timebase = int(str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec))
        
        self.pub_scan.publish(msg)

    def imu_callback(self, msg: Imu):
        msg.angular_velocity.y = -msg.angular_velocity.y
        msg.angular_velocity.z = -msg.angular_velocity.z
        # msg.linear_acceleration.z = -msg.linear_acceleration.z
        
        # msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxLaserToPointcloud()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()