#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.action import Task
from all_seaing_controller.pid_controller import PIDController, CircularPID
from ament_index_python.packages import get_package_share_directory
from all_seaing_interfaces.msg import LabeledBoundingBox2DArray, ControlOption, ObstacleMap
from all_seaing_common.action_server_base import ActionServerBase
from sensor_msgs.msg import CameraInfo

import os
import yaml
import time
import math

class FollowBuoyPID(ActionServerBase):
    def __init__(self):
        super().__init__("follow_path_local_server")

        self._action_server = ActionServer(
            self,
            Task,
            "follow_buoy_path",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,
        )

        self.map_sub = self.create_subscription(
            ObstacleMap,
            "obstacle_map/local",
            self.map_cb,
            10
        )

        self.control_pub = self.create_publisher(
            ControlOption,
            "control_options",
            10
        )

        Kpid_x = (
            self.declare_parameter("Kpid_x", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_y = (
            self.declare_parameter("Kpid_y", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_theta = (
            self.declare_parameter("Kpid_theta", [0.75, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        
        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.prev_update_time = self.get_clock().now()

        self.obstacles = []

        self.timer_period = 1 / 30.0

        self.green_label = 2
        self.red_label = 1

    def set_pid_setpoints(self, x, y, theta):
        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(theta)

    def update_pid(self, x, y, heading):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.x_pid.update(x, dt)
        self.y_pid.update(y, dt)
        self.theta_pid.update(heading, dt)
        self.prev_update_time = self.get_clock().now()

    def dist_squared(self, vect):
        return vect[0] * vect[0] + vect[1] * vect[1]

    def ccw(self, right, yellow, left):
        """Return True if the points a, b, c are counterclockwise, respectively"""
        area = (
            right[0] * yellow[1]
            + yellow[0] * left[1]
            + left[0] * right[1]
            - right[1] * yellow[0]
            - yellow[1] * left[0]
            - left[1] * right[0]
        )
        return area > 0

    def map_cb(self, msg):
        self.obstacles = msg.obstacles

    def control_loop(self):
        # TODO: Fill out the control loop of the code
        raise NotImplementedError

    def execute_callback(self, goal_handle):
        self.start_process("follow buoy pid starting")
        
        # TODO: Fill out setup code before the control loop starts

        while not self.result:

            if self.should_abort():
                self.end_process("aborting follow buoy pid")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("cancelling follow buoy pid")
                goal_handle.canceled()
                return Task.Result()

            self.control_loop()
            time.sleep(self.timer_period)

        self.end_process("follow buoy pid completed!")
        goal_handle.succeed()
        return Task.Result(success=True)



def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPID()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
