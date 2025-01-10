#!/usr/bin/env python3

import rclpy
import numpy as np
import tf_transformations as tfs
from math import pi
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped


class OdometryConverter(object):

    def __init__(self, node, frame_id_in_, frame_id_out_, broadcast_tf_, body_frame_id_, intermediate_frame_id_, world_frame_id_):
        self.node = node  # 使用外部传递的 node 实例
        self.frame_id_in = frame_id_in_
        self.frame_id_out = frame_id_out_
        self.broadcast_tf = broadcast_tf_
        self.body_frame_id = body_frame_id_
        self.intermediate_frame_id = intermediate_frame_id_
        self.world_frame_id = world_frame_id_
        
        # 创建发布者和订阅者
        self.out_odom_pub = self.node.create_publisher(Odometry, 'out_odom', 10)
        self.out_path_pub = self.node.create_publisher(Path, 'out_path', 10)
        self.in_odom_sub = self.node.create_subscription(Odometry, 'in_odom', self.in_odom_callback, 10)
        
        # 创建定时器
        self.path_pub_timer = self.node.create_timer(1.0, self.path_pub_callback)
        self.tf_pub_timer = self.node.create_timer(1.0, self.tf_pub_callback)
        
        self.tf_pub_flag = True
        self.br = TransformBroadcaster(self.node)
        
        if self.broadcast_tf:
            self.node.get_logger().info(f'ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] TF: [{self.body_frame_id}]-[{self.intermediate_frame_id}]-[{self.world_frame_id}]')
        else:
            self.node.get_logger().info(f'ROSTopic: [{self.frame_id_in}]->[{self.frame_id_out}] No TF')

        self.path = []

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = tfs.euler_from_quaternion(q, 'rzyx')
        wqb = tfs.quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = tfs.quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        #### odom ####
        odom_msg = in_odom_msg
        assert in_odom_msg.header.frame_id == self.frame_id_in
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        #### tf ####
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False
            if self.frame_id_in != self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = self.frame_id_out
                t.child_frame_id = self.frame_id_in
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                q_identity = tfs.quaternion_from_euler(0.0, 0.0, 0.0, 'rzyx')
                t.transform.rotation.x = q_identity[0]
                t.transform.rotation.y = q_identity[1]
                t.transform.rotation.z = q_identity[2]
                t.transform.rotation.w = q_identity[3]
                self.br.sendTransform(t)

            if self.world_frame_id != self.frame_id_out:
                t = TransformStamped()
                t.header.stamp = self.node.get_clock().now().to_msg()
                t.header.frame_id = self.frame_id_out
                t.child_frame_id = self.world_frame_id
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q_identity[0]
                t.transform.rotation.y = q_identity[1]
                t.transform.rotation.z = q_identity[2]
                t.transform.rotation.w = q_identity[3]
                self.br.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame_id
            t.child_frame_id = self.body_frame_id
            t.transform.translation.x = p[0]
            t.transform.translation.y = p[1]
            t.transform.translation.z = p[2]
            t.transform.rotation.x = wqb[0]
            t.transform.rotation.y = wqb[1]
            t.transform.rotation.z = wqb[2]
            t.transform.rotation.w = wqb[3]
            self.br.sendTransform(t)

            t = TransformStamped()
            t.header.stamp = self.node.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame_id
            t.child_frame_id = self.intermediate_frame_id
            t.transform.translation.x = p[0]
            t.transform.translation.y = p[1]
            t.transform.translation.z = p[2]
            t.transform.rotation.x = wqc[0]
            t.transform.rotation.y = wqc[1]
            t.transform.rotation.z = wqc[2]
            t.transform.rotation.w = wqc[3]
            self.br.sendTransform(t)

        #### path ####
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path.append(pose)

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000::1]
            self.out_path_pub.publish(path)

    def tf_pub_callback(self):
        self.tf_pub_flag = True

if __name__ == "__main__":
    # 初始化 ROS 2 节点
    rclpy.init()
    node = rclpy.create_node('tf_assist')

    converters = []
    index = 0

    while True:
        prefix = f"converter{index}/"

        try:
            # 从参数服务器获取参数
            frame_id_in = node.declare_parameter(f'{prefix}frame_id_in').value
            frame_id_out = node.declare_parameter(f'{prefix}frame_id_out').value
            broadcast_tf = node.declare_parameter(f'{prefix}broadcast_tf', False).value
            body_frame_id = node.declare_parameter(f'{prefix}body_frame_id', 'body').value
            intermediate_frame_id = node.declare_parameter(f'{prefix}intermediate_frame_id', 'intermediate').value
            world_frame_id = node.declare_parameter(f'{prefix}world_frame_id', 'world').value

            # 初始化 OdometryConverter
            converter = OdometryConverter(
                node, frame_id_in, frame_id_out, broadcast_tf, body_frame_id, intermediate_frame_id, world_frame_id
            )

            # 创建订阅者和发布者
            converter.in_odom_sub = node.create_subscription(
                Odometry, f'{prefix}in_odom', converter.in_odom_callback, 10)
            converter.out_odom_pub = node.create_publisher(
                Odometry, f'{prefix}out_odom', 10)
            converter.out_path_pub = node.create_publisher(
                Path, f'{prefix}out_path', 10)

            # 创建定时器
            converter.tf_pub_timer = node.create_timer(
                0.1, converter.tf_pub_callback)
            converter.path_pub_timer = node.create_timer(
                0.5, converter.path_pub_callback)

            converters.append(converter)
            index += 1

        except Exception as e:
            # 如果没有更多的参数，跳出循环
            if index == 0:
                raise e
            else:
                if index == 1:
                    node.get_logger().info(
                        f'Prefix: "{prefix}" not found. Generated {index} converter.')
                else:
                    node.get_logger().info(
                        f'Prefix: "{prefix}" not found. Generated {index} converters.')
                break

    # 初始化 TF 广播器
    br = TransformBroadcaster(node)

    # 保持节点运行
    rclpy.spin(node)

    # 清理
    node.destroy_node()
    rclpy.shutdown()