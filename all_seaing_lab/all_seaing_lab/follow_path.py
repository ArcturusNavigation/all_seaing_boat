#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

class PathFollower(Node):
    def __init__(self):
        super().__init__('follow_path')

        self.prev_update_time = self.get_clock().now()

        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.red_subscription = self.create_subscription(
            Marker,
            'red_buoys',
            self.red_buoys_cb,
            10
        )
        self.green_subscription = self.create_subscription(
            Marker,
            'green_buoys',
            self.green_buoys_cb,
            10
        )
        self.boat_subscription = self.create_subscription(
            PoseStamped,
            'boat_pose',
            self.boat_callback,
            10
        )

        self.boat_pose = Pose()
        self.red_buoys = []
        self.green_buoys = []
        
        timer_period = 0.1  # seconds
        self.task_completed = False
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.global_frame_id = "map"

        # Keep track of the index of the current pair of buoys
        self.target_buoy = 0

        # Radius around target point where boat is considered to have reached waypoint
        self.acceptance_radius = 0.4

    def red_buoys_cb(self, msg):
        """
        Get the positions of the red buoys
        """
        self.red_buoys = msg.points

    def green_buoys_cb(self, msg):
        """
        Get the positions of the green buoys
        """
        self.green_buoys = msg.points

    def boat_callback(self, msg):
        """
        Get the position of the boat
        """
        self.boat_pose = msg.pose

    def _get_target_pos(self):
        """
        Given the current pair of buoys, return a Point message with the position in their midpoints
        """
        ind = self.target_buoy
        red_buoy = (self.red_buoys[ind].x,  self.red_buoys[ind].y)
        green_buoy = (self.green_buoys[ind].x, self.green_buoys[ind].y)
        target_pose = Point()
        target_pose.x = (red_buoy[0] + green_buoy[0])/2
        target_pose.y = (red_buoy[1] + green_buoy[1])/2
        return target_pose

    def timer_callback(self):
        """
        Follow the Path!
        """
        if (len(self.red_buoys) == 0 or len(self.green_buoys) == 0) or self.task_completed:
            # buoys have not been populated yet
            return
        
        target_pos = self._get_target_pos()
        if (target_pos.x-self.boat_pose.position.x)**2 + (target_pos.y-self.boat_pose.position.y)**2 < self.acceptance_radius ** 2:
            # transition to next waypoint
            self.target_buoy += 1

        if self.target_buoy == len(self.red_buoys):
            # reached the end buoy, so no need to send new goals
            self.task_completed = True
            self.get_logger().info("Finished following the path!")
            return
        
        target_pos = self._get_target_pos()
        target_pose = PoseStamped()
        target_pose.header = Header(frame_id=self.global_frame_id)
        target_pose.pose.position = target_pos
        self.goal_publisher.publish(target_pose)


def main(args=None):
    rclpy.init(args=args)
    path_follower = PathFollower()
    rclpy.spin(path_follower)
    PathFollower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()