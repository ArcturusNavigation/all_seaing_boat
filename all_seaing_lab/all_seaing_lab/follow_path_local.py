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
        self.max_vel = (
            self.declare_parameter("max_vel", [2.0, 2.0, 0.4])
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("forward_speed", 5.0)
        self.declare_parameter("max_yaw", 1.0)
        self.forward_speed = self.get_parameter("forward_speed").get_parameter_value().double_value
        self.max_yaw_rate = self.get_parameter("max_yaw").get_parameter_value().double_value

        self.x_pid = PIDController(*Kpid_x)
        self.y_pid = PIDController(*Kpid_y)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_min(-self.max_vel[2])
        self.theta_pid.set_effort_max(self.max_vel[2])
        self.prev_update_time = self.get_clock().now()

        # update from subs
        self.bboxes = [] # To be commented out
        self.obstacles = []

        self.timer_period = 1 / 30.0

        self.green_labels = set()
        self.red_labels = set()
        self.yellow_labels = set()

        self.result = False
        self.seen_first_buoy = False

        self.scale_right = 1.0

        self.declare_parameter("right_color", "green")
        # Integers, feet apart
        self.declare_parameter("max_distance_apart", 3)
        self.declare_parameter("min_distance_apart", 2)
        self.declare_parameter("meters_feet_conversion", 0.3048)
        self.declare_parameter("front_limit", 0.0)
        self.waypoint_x = None
        self.waypoint_y = None
        self.red_green_ratio = None

        self.right_color = self.get_parameter("right_color").get_parameter_value().string_value
        self.max_distance_apart = self.get_parameter("max_distance_apart").get_parameter_value().integer_value
        self.min_distance_apart = self.get_parameter("min_distance_apart").get_parameter_value().integer_value
        self.meters_feet_conversion = self.get_parameter("meters_feet_conversion").get_parameter_value().double_value
        self.front_limit = self.get_parameter("front_limit").get_parameter_value().double_value

        self.green_labels.add(2)
        self.red_labels.add(1)

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

    def scale_thrust(self, x_vel, y_vel):
        if abs(x_vel) <= self.max_vel[0] and abs(y_vel) <= self.max_vel[1]:
            return x_vel, y_vel

        scale = min(self.max_vel[0] / abs(x_vel), self.max_vel[1] / abs(y_vel))
        return scale * x_vel, scale * y_vel

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

    # Replaced by obstacle map
    def bbox_callback(self, msg):
        self.bboxes = msg.boxes

    # New version with obstacles
    def map_cb(self, msg):
        self.obstacles = msg.obstacles

    def control_loop(self):
        # Access point through name.point.x, etc.?
        red_location = None
        red_dist = 100000

        green_location = None
        green_dist = 100000

        green_x = None
        red_x = None
        green_y = None
        red_y = None

        self.get_logger().info(f'# obstacles: {len(self.obstacles)}')
        for obs in self.obstacles:
            location = obs.local_point
            if location.point.x < 0.5:
                continue
            dist = math.sqrt(location.point.x**2 + location.point.y**2)
            if obs.label in self.green_labels and dist < green_dist:
                self.get_logger().info('GREEN THERE')
                green_dist = dist
                green_location = location
            elif obs.label in self.red_labels and dist < red_dist:
                self.get_logger().info('RED THERE')
                red_dist = dist
                red_location = location

        if green_location is not None and red_location is not None:
            green_y = green_location.point.y
            green_x = green_location.point.x
            red_y = red_location.point.y
            red_x = red_location.point.x

            self.waypoint_y = (red_y + green_y)/2
            self.waypoint_x = (red_x + green_x)/2

        if green_x is None and red_x is None:
            if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 5.0:
                self.get_logger().info("no more buoys killing")
                # wait 1 second, then send a stopping control msg (in case we haven't fully passed the buoys)
                time.sleep(1)

                self.result = True
            self.get_logger().info('KILLING THRUSTERS')
            control_msg = ControlOption()
            control_msg.priority = 1
            self.get_logger().info(f'SENDING COMMAND: {control_msg}')
            self.control_pub.publish(control_msg)
            return
        else:
            self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9

        # POINTS TO GIVE PID: self.waypoint_y and self.waypoint_x
        self.get_logger().info(f"green x: {green_x}, green y: {green_y}")
        self.get_logger().info(f"red x: {red_x}, red y: {red_y}")
        self.get_logger().info(f"waypoint x: {self.waypoint_x}, waypoint y: {self.waypoint_y}")

        self.update_pid(-self.waypoint_x, -self.waypoint_y, math.atan2(-self.waypoint_y, -self.waypoint_x))
        x_output = self.x_pid.get_effort()
        y_output = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output
        y_vel = y_output

        x_vel, y_vel = self.scale_thrust(x_vel, y_vel)
        control_msg = ControlOption()
        control_msg.priority = 1  # Second highest priority, TeleOp takes precedence
        control_msg.twist.linear.x = x_vel
        control_msg.twist.linear.y = y_vel
        control_msg.twist.angular.z = theta_output
        self.control_pub.publish(control_msg)

    def execute_callback(self, goal_handle):
        self.start_process("follow buoy pid starting")
        self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9
        self.set_pid_setpoints(0, 0, 0) #want angle to point to be 0 radians

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
