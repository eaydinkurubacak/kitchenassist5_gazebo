#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ArmMoveSequence(Node):
    def __init__(self):
        super().__init__('arm_move_sequence')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory')
        self.joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.q_start = [0.0, 0.0, 0.2618, 0.0, 0.0]
        self.q1      = [0.0, -0.275, 0.2618, 0.75, 0.0]
        self.q2      = [0.5, 0.5, -0.5, 0.75, 0.0]
        self.q3      = [0.0, 1.0, 0.5, -1.5, 0.0]
        self.sequence = [self.q1, self.q2, self.q3, self.q_start]

    def send_goal(self, positions, sec=3):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=sec, nanosec=0)
        goal.trajectory.points = [pt]
        self.client.wait_for_server()
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected')
            return
        rclpy.spin_until_future_complete(self, gh.get_result_async())
        self.get_logger().info(f'Reached: {positions}')

    def run(self):
        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()
        self.get_logger().info('Starting sequence from q_start')
        time.sleep(2.0)
        for i, q in enumerate(self.sequence):
            self.get_logger().info(f'Step {i+1}/{len(self.sequence)} -> {q}')
            self.send_goal(q)
            time.sleep(1.5)
            self.get_logger().info('---')


def main():
    rclpy.init()
    node = ArmMoveSequence()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
