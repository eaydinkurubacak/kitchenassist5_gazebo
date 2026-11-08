#!/usr/bin/env python3
import rclpy.parameter
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped


class EndEffectorPublisher(Node):
    def __init__(self):
        super().__init__('end_effector_publisher')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PointStamped, '/end_effector_position', 10)
        self.timer = self.create_timer(0.1, self.publish_position)

    def publish_position(self):
        try:
            t = self.buffer.lookup_transform(
                'base_link', 'tool_frame', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warning(f'TF lookup failed: {e}')
            return

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = t.transform.translation.x
        msg.point.y = t.transform.translation.y
        msg.point.z = t.transform.translation.z
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = EndEffectorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
