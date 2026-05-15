#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class OdomToMap(Node):
    def __init__(self):
        super().__init__('odom_to_map')

        self.declare_parameter('robot_id', 0)

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().integer_value

        self.from_frame = f'r{self.robot_id}/odom'
        self.to_frame = f'robot{self.robot_id}_map'

        self.sub = self.create_subscription(TFMessage, f'/r{self.robot_id}/tf', self._cb, 10)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 10)
        self.pub_rN = self.create_publisher(TFMessage, f'/r{self.robot_id}/tf', 10)

        self.get_logger().info(
            f'Listening /r{self.robot_id}/tf -> /tf ({self.from_frame}->{self.to_frame}, wall clock time), also republishing to /r{self.robot_id}/tf')

    def _cb(self, msg: TFMessage):
        now = self.get_clock().now()

        out_tf = TFMessage()
        out_rN = TFMessage()

        for t in msg.transforms:
            t_tf = TransformStamped()
            t_tf.header.stamp = now.to_msg()
            t_tf.header.frame_id = self.to_frame if t.header.frame_id == self.from_frame else t.header.frame_id
            t_tf.child_frame_id = self.to_frame if t.child_frame_id == self.from_frame else t.child_frame_id
            t_tf.transform = t.transform
            out_tf.transforms.append(t_tf)

            t_r0 = TransformStamped()
            t_r0.header.stamp = now.to_msg()
            t_r0.header.frame_id = t.header.frame_id
            t_r0.child_frame_id = t.child_frame_id
            t_r0.transform = t.transform
            out_rN.transforms.append(t_r0)

        if out_tf.transforms:
            self.pub_tf.publish(out_tf)
            self.pub_rN.publish(out_rN)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdomToMap())
    rclpy.shutdown()


if __name__ == '__main__':
    main()