#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class StaticTransform(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')

        self.declare_parameter('frame_id', 'robot0_current_pose')
        self.declare_parameter('child_frame', 'r0/base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter("source_tf", "/tf")
        self.declare_parameter("target_tf", "/r0/tf")
        self.declare_parameter("source_qos", 10)
        self.declare_parameter("target_qos", 10)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        self.source_tf = str(self.get_parameter("source_tf").value)
        self.target_tf = str(self.get_parameter("target_tf").value)
        self.source_qos = self.get_parameter('source_qos').get_parameter_value().integer_value
        self.target_qos = self.get_parameter('target_qos').get_parameter_value().integer_value

        rate = float(self.get_parameter('rate').value)

        self.pub = self.create_publisher(TFMessage, self.source_tf, self.source_qos)
        self.pub_2 = self.create_publisher(TFMessage, self.target_tf, self.target_qos)

        period = 1.0 / rate
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'Publishing: {self.frame_id} -> {self.child_frame} at {rate} Hz to {self.source_tf} and {self.target_tf}')

    def _publish(self):
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        msg = TFMessage(transforms=[t])
        self.pub.publish(msg)
        self.pub_2.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StaticTransform())
    rclpy.shutdown()


if __name__ == '__main__':
    main()