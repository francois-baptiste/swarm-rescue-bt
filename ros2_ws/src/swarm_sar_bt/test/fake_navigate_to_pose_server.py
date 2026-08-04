#!/usr/bin/env python3
"""Stub `navigate_to_pose` action server, for runtime smoke-testing
mission_bt_node without a full Nav2 stack (no costmap/planner/controller/
Gazebo required).

This is a TEST DOUBLE, not part of the package's real Nav2 integration:
the NavigateToPose BT node compiled into mission_bt_node is Nav2's own
real code (nav2_navigate_to_pose_action_bt_node) - it talks to this stub
exactly as it would talk to a real bt_navigator, over the same
nav2_msgs/action/NavigateToPose action. Only the server side is faked
here (straight-line motion at a fixed speed instead of real planning and
control), so running mission_bt_node against this exercises the BT
plugins, the SwarmKnowledge decentralized coordination and the real
NavigateToPose action-client wiring for real, just not real navigation.
"""
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster


class FakeNavigateToPoseServer(Node):
    def __init__(self):
        super().__init__('fake_navigate_to_pose_server')
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('speed', 2.0)  # m/s
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')

        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.speed = float(self.get_parameter('speed').value)
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value

        cb_group = ReentrantCallbackGroup()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_tf, callback_group=cb_group)
        self._server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose', self.execute_callback,
            callback_group=cb_group)

        self.get_logger().info(
            f'fake_navigate_to_pose_server up at ({self.x:.2f}, {self.y:.2f})')

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.global_frame
        t.child_frame_id = self.robot_base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def execute_callback(self, goal_handle):
        goal_x = goal_handle.request.pose.pose.position.x
        goal_y = goal_handle.request.pose.pose.position.y
        self.get_logger().info(f'navigate_to_pose goal: ({goal_x:.2f}, {goal_y:.2f})')

        rate_hz = 10.0
        step = self.speed / rate_hz
        rate = self.create_rate(rate_hz)
        while rclpy.ok():
            dx, dy = goal_x - self.x, goal_y - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.15:
                self.x, self.y = goal_x, goal_y
                break
            if step >= dist:
                self.x, self.y = goal_x, goal_y
            else:
                self.x += dx / dist * step
                self.y += dy / dist * step
            rate.sleep()

        goal_handle.succeed()
        self.get_logger().info(f'navigate_to_pose reached ({self.x:.2f}, {self.y:.2f})')
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNavigateToPoseServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
