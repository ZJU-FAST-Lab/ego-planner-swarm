#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations as tfs
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Joy

pub = None
pub1 = None
pub2 = None

def callback(odom_msg):
    q = np.array([odom_msg.pose.pose.orientation.x,
                  odom_msg.pose.pose.orientation.y,
                  odom_msg.pose.pose.orientation.z,
                  odom_msg.pose.pose.orientation.w])

    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = odom_msg.header
    euler_msg.vector.z = e[0] * 180.0 / 3.14159
    euler_msg.vector.y = e[1] * 180.0 / 3.14159
    euler_msg.vector.x = e[2] * 180.0 / 3.14159

    pub.publish(euler_msg)

def imu_callback(imu_msg):
    q = np.array([imu_msg.orientation.x,
                  imu_msg.orientation.y,
                  imu_msg.orientation.z,
                  imu_msg.orientation.w])

    e = tfs.euler_from_quaternion(q, 'rzyx')

    euler_msg = Vector3Stamped()
    euler_msg.header = imu_msg.header
    euler_msg.vector.z = e[0] * 180.0 / 3.14159
    euler_msg.vector.y = e[1] * 180.0 / 3.14159
    euler_msg.vector.x = e[2] * 180.0 / 3.14159

    pub1.publish(euler_msg)

def joy_callback(joy_msg):
    out_msg = Vector3Stamped()
    out_msg.header = joy_msg.header
    out_msg.vector.z = -joy_msg.axes[3]
    out_msg.vector.y = joy_msg.axes[1]
    out_msg.vector.x = joy_msg.axes[0]

    pub2.publish(out_msg)


if __name__ == "__main__":
    # 初始化 ROS 2 节点
    rclpy.init()
    node = rclpy.create_node("odom_to_euler")

    # 创建发布者和订阅者
    pub = node.create_publisher(Vector3Stamped, "euler", 10)
    sub = node.create_subscription(Odometry, "odom", callback, 10)

    pub1 = node.create_publisher(Vector3Stamped, "imueuler", 10)
    sub1 = node.create_subscription(Imu, "imu", imu_callback, 10)

    pub2 = node.create_publisher(Vector3Stamped, "ctrlout", 10)
    sub2 = node.create_subscription(Joy, "ctrlin", joy_callback, 10)

    # 保持节点运行
    rclpy.spin(node)

    # 关闭节点
    node.destroy_node()
    rclpy.shutdown()