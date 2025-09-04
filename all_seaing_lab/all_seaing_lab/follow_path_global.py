#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import FollowPath, Task, Waypoint
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import math
import time

class FollowBuoyPath(ActionServerBase):
    def __init__(self):
        super().__init__("follow_path_global_server")

        self._action_server = ActionServer(
            self,
            Task,
            "follow_buoy_path",
            execute_callback=self.execute_callback,
            cancel_callback=self.default_cancel_callback,

        )

        self.map_sub = self.create_subscription(
            ObstacleMap, "obstacle_map/global", self.map_cb, 10
        )
        self.follow_path_client = ActionClient(self, FollowPath, "follow_path")
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray, "waypoint_markers", 10
        )

        self.declare_parameter("xy_threshold", 2.0)
        self.declare_parameter("theta_threshold", 180.0)
        self.declare_parameter("goal_tol", 0.5)
        self.declare_parameter("obstacle_tol", 50)
        self.declare_parameter("choose_every", 1)
        self.declare_parameter("use_waypoint_client", False)
        self.declare_parameter("planner", "astar")

        # TODO: Add custom parameters

        self.timer_period = 1/60

        self.obstacles = None

        self.green_label = 2
        self.red_label = 1

        # TODO: Fill out init code

    def norm_squared(self, vec, ref=(0, 0)):
        return vec[0] ** 2 + vec[1] ** 2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def ccw(self, a, b, c):
        """Return True if the points a, b, c are counterclockwise, respectively"""
        area = (
            a[0] * b[1]
            + b[0] * c[1]
            + c[0] * a[1]
            - a[1] * b[0]
            - b[1] * c[0]
            - c[1] * a[0]
        )
        return area > 0
    
    def send_waypoint_to_server(self, waypoint):
        # sending waypoints to navigation server
        self.follow_path_client.wait_for_server()
        goal_msg = FollowPath.Goal()
        goal_msg.planner = self.get_parameter("planner").value
        self.get_logger().info(f'Sending waypoint: {waypoint}')
        goal_msg.x = waypoint[0]
        goal_msg.y = waypoint[1]
        goal_msg.xy_threshold = self.get_parameter("xy_threshold").value
        goal_msg.theta_threshold = self.get_parameter("theta_threshold").value
        goal_msg.goal_tol = self.get_parameter("goal_tol").value
        goal_msg.obstacle_tol = self.get_parameter("obstacle_tol").value
        goal_msg.choose_every = self.get_parameter("choose_every").value
        goal_msg.is_stationary = True
        self.follow_path_client.wait_for_server()
        self.send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg
        )

    def map_cb(self, msg):
        self.obstacles = msg.obstacles        

    def execute_callback(self, goal_handle):

        self.start_process("Follow buoy path started!")

        while rclpy.ok() and self.obstacles is None:
            time.sleep(0.2)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
        
        success = False
        while not success:
            # Check if we should abort/cancel if a new goal arrived
            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return Task.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
            
            # TODO: Fill out buoy setup code

            time.sleep(0.1)

        self.get_logger().info("Setup buoys succeeded!")

        while not self.result:
            # Check if we should abort/cancel if a new goal arrived
            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return Task.Result()

            if goal_handle.is_cancel_requested:
                self.end_process("Cancel requested. Aborting path following.")
                goal_handle.canceled()
                return Task.Result()
            
            # TODO: Fill out path following code (that runs after setup)

            time.sleep(self.timer_period)

        self.end_process("Follow buoy path completed!")
        goal_handle.succeed()
        return Task.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = FollowBuoyPath()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
