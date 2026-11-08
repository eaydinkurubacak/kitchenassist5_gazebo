#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped
import numpy as np
import math
import time


def dh_transform(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,      d],
        [0,         0,        0,      1],
    ])

"""
def fk(q):
    d1, a2, a3, d5 = 0.64, 0.30, 0.25, 0.05
    a1_alpha = -math.pi / 2
    a4_alpha = -math.pi / 2
    T = np.eye(4)
    T = T @ dh_transform(q[0], d1, 0.0, a1_alpha)
    T = T @ dh_transform(q[1], 0.0, a2, 0.0)
    T = T @ dh_transform(q[2], 0.0, a3, 0.0)
    T = T @ dh_transform(q[3], 0.0, 0.0, a4_alpha)
    T = T @ dh_transform(q[4], d5, 0.0, 0.0)
    return T[:3, 3]
"""
def fk(q):
    d1, a2, a3, d5 = 0.64, 0.30, 0.25, 0.05
    a1_alpha = -math.pi / 2
    a4_alpha = -math.pi / 2
    
    T = np.eye(4)
    T = T @ dh_transform(q[0], d1, 0.0, a1_alpha)
    T = T @ dh_transform(q[1], 0.0, a2, 0.0)
    T = T @ dh_transform(q[2], 0.0, a3, 0.0)
    T = T @ dh_transform(q[3], 0.0, 0.0, a4_alpha)
    
    # ------------------------------------------------------------------
    # JOINT 5 SABİT RPY DÖNÜŞÜMÜ (Xacro: rpy="${-90*DEG2RAD} 0 ${-90*DEG2RAD}")
    # ------------------------------------------------------------------
    # Roll (-90 deg) ve Yaw (-90 deg) dönüşüm matrisleri:
    R_x_m90 = np.array([
        [1,  0,  0, 0],
        [0,  0,  1, 0],
        [0, -1,  0, 0],
        [0,  0,  0, 1]
    ])
    
    R_z_m90 = np.array([
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 1, 0],
        [ 0, 0, 0, 1]
    ])
    
    # URDF sabit dönüşümünü ekliyoruz (R_z * R_x düzeni):
    T = T @ R_z_m90 @ R_x_m90
    
    # J5 eklem açısı hareketi ve d5 ötelemesi
    T = T @ dh_transform(q[4], d5, 0.0, 0.0)
    
    return T[:3, 3]

class FKValidation(Node):
    def __init__(self):
        super().__init__('fk_validation')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory')
        self.joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        self.ee_pos = None
        self.create_subscription(PointStamped, '/end_effector_position',
                                 self.ee_cb, 10)
        
        self.configs = [
            [0.0,   0.0,   0.2618, 0.0,  0.0],
            [0.3,  -0.3,   0.2,    0.5,  0.0],
            [-0.3,  0.3,  -0.3,    0.5,  0.5],
            [0.5,  -0.5,   0.1,    0.7, -0.5],
            [-0.5,  0.4,  -0.5,    0.4,  1.0],
            [0.7,  -0.7,   0.0,    0.3,  0.0],
            [-0.7,  0.5,   0.3,    0.6, -0.8],
            [0.2,  -0.2,  -0.4,    0.2,  0.3],
            [-0.2,  0.2,   0.4,    0.8, -0.3],
            [0.0,   0.0,   0.2618, 0.0,  0.0],
        ]

    def ee_cb(self, msg):
        self.ee_pos = np.array([msg.point.x, msg.point.y, msg.point.z])

    def send_goal(self, positions, sec=3):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=sec, nanosec=0)
        goal.trajectory.points = [pt]
        self.client.wait_for_server()
        f = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected')
            return
        rclpy.spin_until_future_complete(self, gh.get_result_async())

    def run(self):
        print("=== waiting for action server ===", flush=True)
        self.client.wait_for_server()
        errors = []
        for i, q in enumerate(self.configs):
            self.get_logger().info(f'Config {i+1}/10: {q}')
            self.send_goal(q)
            time.sleep(2.0)
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.05)            
            # /end_effector_position güncel değeri al
            self.ee_pos = None
            t0 = time.time()
            while self.ee_pos is None and time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.ee_pos is None:
                self.get_logger().warn('No EE position received')
                continue
            fk_pos = fk(q)
            err = float(np.linalg.norm(fk_pos - self.ee_pos))
            errors.append(err)
            self.get_logger().info(
                f'  FK={fk_pos.round(4)}  ROS={self.ee_pos.round(4)}  err={err:.6f} m')
        if errors:
            arr = np.array(errors)
            self.get_logger().info('===== RESULTS =====')
            self.get_logger().info(f'N = {len(arr)}')
            self.get_logger().info(f'Mean error = {arr.mean():.6f} m')
            self.get_logger().info(f'Std  error = {arr.std():.6f} m')
            self.get_logger().info(f'Max  error = {arr.max():.6f} m')


def main():
    rclpy.init()
    node = FKValidation()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("=== fk_validation started ===", flush=True)
    main()
