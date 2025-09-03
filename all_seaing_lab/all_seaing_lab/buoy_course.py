#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Point, PointStamped, PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np

from all_seaing_interfaces.msg import ObstacleMap, Obstacle

class BuoyCourse(Node):
    def __init__(self):
        super().__init__('buoy_course')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('red_buoys_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('red_buoys_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('green_buoys_x', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('green_buoys_y', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )

        self.global_frame_id = "map"
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        self.red_publisher = self.create_publisher(Marker, 'red_buoys', 10)
        self.green_publisher = self.create_publisher(Marker, 'green_buoys', 10)
        self.pose_sub = self.create_subscription(PoseStamped, 'boat_pose', self.pose_cb, 10)
        self.local_publisher = self.create_publisher(ObstacleMap, 'obstacle_map/local', 10)
        self.global_publisher = self.create_publisher(ObstacleMap, 'obstacle_map/global', 10)
        
        self.nav_x = 0.0
        self.nav_y = 0.0
        self.nav_heading = 0.0

    def timer_callback(self):
        red_buoys_x = self.get_parameter('red_buoys_x').value
        red_buoys_y = self.get_parameter('red_buoys_y').value
        green_buoys_x = self.get_parameter('green_buoys_x').value
        green_buoys_y = self.get_parameter('green_buoys_y').value

        red_marker = self._buoys_to_marker(red_buoys_x, red_buoys_y, "red")
        green_marker = self._buoys_to_marker(green_buoys_x, green_buoys_y, "green")
        self.publish_map(red_buoys_x, red_buoys_y, green_buoys_x, green_buoys_y)

        self.red_publisher.publish(red_marker)
        self.green_publisher.publish(green_marker)

    def pose_cb(self, pose_msg: PoseStamped):
        self.nav_x = pose_msg.pose.position.x
        self.nav_y = pose_msg.pose.position.y
        self.nav_heading = euler_from_quaternion([pose_msg.pose.orientation.x,
                                                 pose_msg.pose.orientation.y,
                                                 pose_msg.pose.orientation.z,
                                                 pose_msg.pose.orientation.w])[2]
    
    def publish_map(self, red_buoys_x, red_buoys_y, green_buoys_x, green_buoys_y):
        obs_map = ObstacleMap()
        obs_map.header.stamp = self.get_clock().now().to_msg()
        obs_map.header.frame_id = "map"
        obs_map.local_header.stamp = self.get_clock().now().to_msg()
        obs_map.local_header.frame_id = "base_link"
        local_obs_map = ObstacleMap()
        local_obs_map.header = obs_map.header
        local_obs_map.local_header = obs_map.local_header
        id = 0
        for buoy_x, buoy_y in zip(red_buoys_x, red_buoys_y):
            obs_map.obstacles.append(self.buoy_to_global_obstacle(buoy_x, buoy_y, 1, id))
            local_obs_map.obstacles.append(self.buoy_to_local_obstacle(buoy_x, buoy_y, 1, id))
            id += 1
        for buoy_x, buoy_y in zip(green_buoys_x, green_buoys_y):
            obs_map.obstacles.append(self.buoy_to_global_obstacle(buoy_x, buoy_y, 2, id))
            local_obs_map.obstacles.append(self.buoy_to_local_obstacle(buoy_x, buoy_y, 2, id))
            id += 1
        
        self.local_publisher.publish(local_obs_map)
        self.global_publisher.publish(obs_map)

    def _buoys_to_marker(self, buoys_x, buoys_y, color):
        """
        Helper function to convert buoys to marker of type sphere list.
        @param buoys_x x coordinates of buoys
        @param buoys_y y coordinates of buoys
        @param color the color of the marker
        """
        marker = Marker()
        marker.header = Header(frame_id=self.global_frame_id)
        marker.type = Marker.SPHERE_LIST
        marker.scale = Vector3(x=0.3, y=0.3, z=0.2)
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        else:
            marker.color.b = 1.0

        for buoy_x, buoy_y in zip(buoys_x, buoys_y):
            marker.points.append(Point(x=buoy_x, y=buoy_y))

        return marker

    def compute_transform_from_to(self, from_pos, to_pos):
        from_x, from_y, from_theta = from_pos
        to_x, to_y, to_theta = to_pos
        dx = np.cos(from_theta)*(to_x-from_x)+np.sin(from_theta)*(to_y-from_y)
        dy = -np.sin(from_theta)*(to_x-from_x)+np.cos(from_theta)*(to_y-from_y)
        dtheta = to_theta - from_theta
        return (dx, dy, dtheta)

    def compose_transforms(self, t1, t2):
        t1_dx, t1_dy, t1_dtheta = t1
        t2_dx, t2_dy, t2_dtheta = t2
        t_dx = t1_dx+np.cos(t1_dtheta)*t2_dx-np.sin(t1_dtheta)*t2_dy
        t_dy = t1_dy+np.sin(t1_dtheta)*t2_dx+np.cos(t1_dtheta)*t2_dy
        t_dtheta = t1_dtheta+t2_dtheta
        return (t_dx, t_dy, t_dtheta)

    def buoy_to_local_obstacle(self, global_x, global_y, label, id):
        # robot->buoy = robot->map->buoy = inv(map->robot)@(map->buoy)
        local_x, local_y, _ = self.compose_transforms(self.compute_transform_from_to((self.nav_x, self.nav_y, self.nav_heading), (0.0,0.0,0.0)),(global_x, global_y, 0.0))
        return Obstacle(id=id, label=label, local_point=PointStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"), point=Point(x=local_x, y=local_y)))
        
    def buoy_to_global_obstacle(self, x, y, label, id):
        obs = self.buoy_to_local_obstacle(x, y, label, id)
        obs.global_point=PointStamped(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="map"), point=Point(x=x, y=y))
        return obs

def main():
    rclpy.init()
    node = BuoyCourse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()