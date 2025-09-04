#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor


from all_seaing_interfaces.msg import ObstacleMap, Obstacle
from all_seaing_interfaces.action import FollowPath, Task, Waypoint
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from all_seaing_common.action_server_base import ActionServerBase
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import math
import os
import yaml
import time

class InternalBuoyPair:
    def __init__(self, left_buoy=None, right_buoy=None):
        if left_buoy is None:
            self.left = Obstacle()
        else:
            self.left = left_buoy

        if right_buoy is None:
            self.right = Obstacle()
        else:
            self.right = right_buoy


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
        self.declare_parameter("planner", "astar")

        self.declare_parameter("buoy_pair_dist_thres", 1.0)
        self.buoy_pair_dist_thres = self.get_parameter("buoy_pair_dist_thres").get_parameter_value().double_value

        self.robot_pos = (0, 0)

        self.declare_parameter("safe_margin", 0.2)

        self.first_buoy_pair = True

        self.safe_margin = (
            self.get_parameter("safe_margin").get_parameter_value().double_value
        )

        self.green_labels = set()
        self.red_labels = set()

        self.green_labels.add(2)
        self.red_labels.add(1)
        
        self.sent_waypoints = set()

        self.red_left = True
        self.result = False
        self.timer_period = 1/60
        self.time_last_seen_buoys = time.time()

        self.obstacles = None

        self.buoy_pairs = []
        self.obstacles = []

        self.thresh_dist = 5.0

        self.sent_forward = False

        self.pair_to = None

    def norm_squared(self, vec, ref=(0, 0)):
        return vec[0] ** 2 + vec[1] ** 2

    def norm(self, vec, ref=(0, 0)):
        return math.sqrt(self.norm_squared(vec, ref))

    def ob_coords(self, buoy, local=False):
        if local:
            return (buoy.local_point.point.x, buoy.local_point.point.y)
        else:
            return (buoy.global_point.point.x, buoy.global_point.point.y)

    def get_closest_to(self, source, buoys, local=False):
        return min(
            buoys,
            key=lambda buoy: math.dist(source, self.ob_coords(buoy, local)),
        )

    def midpoint(self, vec1, vec2):
        return ((vec1[0] + vec2[0]) / 2, (vec1[1] + vec2[1]) / 2)

    def midpoint_pair(self, pair):
        left_coords = self.ob_coords(pair.left)
        right_coords = self.ob_coords(pair.right)
        midpoint = self.midpoint(left_coords, right_coords)
        
        scale = 1 # number of meters to translate forward. TODO: parametrize.
        dy = right_coords[1] - left_coords[1]
        dx = right_coords[0] - left_coords[0]
        norm = math.sqrt(dx**2 + dy**2)
        dx /= norm
        dy /= norm
        midpoint = (midpoint[0] - scale*dy, midpoint[1] + scale*dx)

        return midpoint

    def split_buoys(self, obstacles):
        """
        Splits the buoys into red and green based on their labels in the obstacle map
        """
        green_bouy_points = []
        red_bouy_points = []
        for obstacle in obstacles:
            if obstacle.label in self.green_labels:
                green_bouy_points.append(obstacle)
            elif obstacle.label in self.red_labels:
                red_bouy_points.append(obstacle)
        return green_bouy_points, red_bouy_points

    def obs_to_pos(self, obs):
        return [self.ob_coords(ob, local=False) for ob in obs]

    def obs_to_pos_label(self, obs):
        return [self.ob_coords(ob, local=False) + (ob.label,) for ob in obs]

    def buoy_pairs_to_markers(self, buoy_pairs):
        """
        Create the markers from an array of buoy pairs to visualize them (and the respective waypoints) in RViz
        """
        marker_array = MarkerArray()
        i = 0
        for p_left, p_right, point, radius in buoy_pairs:
            marker_array.markers.append(
                Marker(
                    type=Marker.ARROW,
                    pose=point,
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=2.0, y=0.15, z=0.15),
                    color=ColorRGBA(a=1.0, b=1.0),
                    id=(4 * i),
                )
            )
            if self.red_left:
                left_color = ColorRGBA(r=1.0, a=1.0)
                right_color = ColorRGBA(g=1.0, a=1.0)
            else:
                left_color = ColorRGBA(g=1.0, a=1.0)
                right_color = ColorRGBA(r=1.0, a=1.0)

            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(p_left)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=left_color,
                    id=(4 * i) + 1,
                )
            )
            marker_array.markers.append(
                Marker(
                    type=Marker.SPHERE,
                    pose=self.pair_to_pose(self.ob_coords(p_right)),
                    header=Header(frame_id=self.global_frame_id),
                    scale=Vector3(x=1.0, y=1.0, z=1.0),
                    color=right_color,
                    id=(4 * i) + 2,
                )
            )
            i += 1
        return marker_array
    
    def follow_path(self):
        # get robot position
        x,y,_ = self.get_robot_pose()
        self.robot_pos = (x,y)

        # Check if we passed that pair of buoys (the robot is in front of the pair), then move on to the next one
        left_coords = self.ob_coords(self.pair_to.left)
        right_coords = self.ob_coords(self.pair_to.right)
        x, y = self.midpoint(left_coords, right_coords)
        rx, ry = self.robot_pos
        if self.ccw(
            left_coords,
            right_coords, 
            self.robot_pos,
        ) or (x - rx) ** 2 + (y - ry) ** 2 <= self.thresh_dist:
            if self.find_waypoint():
                self.time_last_seen_buoys = self.get_clock().now().nanoseconds / 1e9
                return True
            else:
                if (self.get_clock().now().nanoseconds / 1e9) - self.time_last_seen_buoys > 5.0:
                    self.result = True
                return False

        else:
            return True

    def find_waypoint(self):
        self.get_logger().info("Finding new follow path waypoint")
        self.get_logger().info(
            f"list of obstacles: {self.obs_to_pos_label(self.obstacles)}"
        )

        # Split all the buoys into red and green
        green_init, red_init = self.split_buoys(self.obstacles)

        # lambda function that filters the buoys that are in front of the robot
        obstacles_in_front = lambda obs: [
            ob for ob in obs
            if ob.local_point.point.x > 0
        ]
        # take the green and red buoys that are in front of the robot
        green_buoys, red_buoys = obstacles_in_front(green_init), obstacles_in_front(red_init)
        self.get_logger().info(
            f"red buoys: {red_buoys}, green buoys: {green_buoys}"
        )
        if len(red_buoys) == 0 or len(green_buoys) == 0:
            self.get_logger().info("No buoy pairs!")
            return False

        # From the red buoys that are in front of the robot, take the one that is closest to it.
        # And do the same for the green buoys.
        closest_red = self.get_closest_to((0, 0), red_buoys, local=True)
        closest_green = self.get_closest_to((0, 0), green_buoys, local=True)
        self.pair_to = InternalBuoyPair(
            closest_red,
            closest_green,
        )

        # send waypoint
        waypoint = self.midpoint_pair(self.pair_to)
        self.send_waypoint_to_server(waypoint)

        # visualize waypoint
        self.waypoint_marker_pub.publish(MarkerArray(markers=[Marker(id=0,action=Marker.DELETEALL)]))
        self.waypoint_marker_pub.publish(self.buoy_pairs_to_markers([(self.pair_to.left, self.pair_to.right, self.pair_angle_to_pose(
            pair=waypoint,
            angle=(
                math.atan(self.ob_coords(self.pair_to.right)[1] - self.ob_coords(self.pair_to.left)[1]) /
                (self.ob_coords(self.pair_to.right)[0] - self.ob_coords(self.pair_to.left)[0])
            ) + (math.pi / 2),
        ), self.norm(self.ob_coords(self.pair_to.left), self.ob_coords(self.pair_to.right))/2 - self.safe_margin)]))
        return True

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

    def filter_front_buoys(self, pair, buoys):
        """
        Returns the buoys (from the given array) that are in front of a pair of points,
        considering the forward direction to be the one such that
        the first point of the pair is in the left and the second is in the right
        """
        # (red, green)
        return [
            buoy
            for buoy in buoys
            if self.ccw(
                self.ob_coords(pair.left),
                self.ob_coords(pair.right),
                self.ob_coords(buoy),
            ) and min(self.norm((self.ob_coords(buoy)[0]-self.ob_coords(pair.left)[0], self.ob_coords(buoy)[1]-self.ob_coords(pair.left)[1])), self.norm((self.ob_coords(buoy)[0]-self.ob_coords(pair.right)[0], self.ob_coords(buoy)[1]-self.ob_coords(pair.right)[1]))) > self.buoy_pair_dist_thres
        ]

    def pair_to_pose(self, pair):
        return Pose(position=Point(x=pair[0], y=pair[1]))
    
    def pair_angle_to_pose(self, pair, angle):
        quat = quaternion_from_euler(0, 0, angle)
        return Pose(
            position=Point(x=pair[0], y=pair[1]),
            orientation=Quaternion(x=quat[0], y=quat[2], z=quat[2], w=quat[3]),
        )
    
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
        """
        When a new map is received, check if it is the first one (we haven't set up the starting buoys)
        and find the starting pair, and then (if the starting buoys are successfully computed) form
        the buoy pair / waypoint sequence
        """
        self.obstacles = msg.obstacles        

    def execute_callback(self, goal_handle):

        self.start_process("Follow buoy path started!")

        while rclpy.ok() and self.obstacles is None:
            time.sleep(0.2) # TODO: maybe change this
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()
        
        success = False
        while not success:
            success = self.find_waypoint()
            time.sleep(0.1)
            # Check if we should abort/cancel if a new goal arrived
            if self.should_abort():
                self.end_process("New request received. Aborting path following.")
                goal_handle.abort()
                return Task.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Task.Result()

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
            
            self.follow_path()

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
