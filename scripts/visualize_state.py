#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cv2
import numpy as np
from collections import deque


class VisualizeState(Node):
    def __init__(self):
        super().__init__('visualize_state')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])

        self.bridge = CvBridge()
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.history_len = 200
        self.history = {j: deque(maxlen=self.history_len) for j in self.joint_names}

        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Image, '/kitchen_camera/image_raw', self.image_cb, 10)
        self.latest_image = None

        # Matplotlib figürü
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.lines = {j: self.ax.plot([], [], label=j)[0] for j in self.joint_names}
        self.ax.set_ylim(-3.2, 3.2)
        self.ax.set_xlim(0, self.history_len)
        self.ax.set_ylabel('rad')
        self.ax.set_title('Joint Positions')
        self.ax.legend(loc='upper right')
        self.ax.grid(True)

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, blit=False, cache_frame_data=False)

    def joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.history:
                self.history[name].append(pos)

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge: {e}')

    def update_plot(self, _):
        for j, line in self.lines.items():
            data = list(self.history[j])
            line.set_data(range(len(data)), data)
        return list(self.lines.values())


def main():
    rclpy.init()
    node = VisualizeState()

    # ROS spin'i ayrı bir thread'de çalıştır
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.show(block=False)

    try:
        while rclpy.ok():
            plt.pause(0.05)
            if node.latest_image is not None:
                cv2.imshow('Kitchen Camera', node.latest_image)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
