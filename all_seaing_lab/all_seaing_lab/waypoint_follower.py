import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Pose
from all_seaing_lab.pid_controller import PIDController, CircularPID
from tf_transformations import euler_from_quaternion
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        Kpid_x = (
            self.declare_parameter("Kpid_x", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_y = (
            self.declare_parameter("Kpid_y", [1.0, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        Kpid_theta = (
            self.declare_parameter("Kpid_theta", [1.3, 0.0, 0.0])
            .get_parameter_value()
            .double_array_value
        )
        self.x_pid = PIDController(*Kpid_x)
        self.x_pid.set_effort_max(1.2)
        self.y_pid = PIDController(*Kpid_y)
        self.y_pid.set_effort_max(1.2)
        self.theta_pid = CircularPID(*Kpid_theta)
        self.theta_pid.set_effort_max(1.5)
        self.prev_update_time = self.get_clock().now()

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10
        )
        self.boat_subscription = self.create_subscription(
            PoseStamped,
            'boat_pose',
            self.boat_callback,
            10
        )

        self.goal_pose = Pose()
        self.boat_pose = Pose()
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def boat_callback(self, msg):
        self.boat_pose = msg.pose

    def set_pid_setpoints(self, x, y, theta):
        self.x_pid.set_setpoint(x)
        self.y_pid.set_setpoint(y)
        self.theta_pid.set_setpoint(theta)

    def reset_pid(self):
        self.prev_update_time = self.get_clock().now()
        self.x_pid.reset()
        self.y_pid.reset()
        self.theta_pid.reset()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose
        goal_pos = self.goal_pose.position
        self.reset_pid()
        self.set_pid_setpoints(
            goal_pos.x, 
            goal_pos.y, 
            math.atan2(goal_pos.y - self.boat_pose.position.y, goal_pos.x - self.boat_pose.position.x)
        )

    def update_pid(self, x, y, heading):
        dt = (self.get_clock().now() - self.prev_update_time).nanoseconds / 1e9
        self.theta_pid.set_setpoint(
            math.atan2(self.goal_pose.position.y - self.boat_pose.position.y, 
                       self.goal_pose.position.x - self.boat_pose.position.x))
        self.x_pid.update(x, dt)
        self.y_pid.update(y, dt)
        self.theta_pid.update(heading, dt)
        self.prev_update_time = self.get_clock().now()

    def timer_callback(self):
        # Feed the new boat position to the PID controllers and calculate the new velocities
        nav_x = self.boat_pose.position.x
        nav_y = self.boat_pose.position.y
        heading = euler_from_quaternion([
            self.boat_pose.orientation.x,
            self.boat_pose.orientation.y,
            self.boat_pose.orientation.z,
            self.boat_pose.orientation.w,
        ])[2]
        self.update_pid(nav_x, nav_y, heading)
        x_output = self.x_pid.get_effort()
        y_output = self.y_pid.get_effort()
        theta_output = self.theta_pid.get_effort()
        x_vel = x_output * math.cos(heading) + y_output * math.sin(heading)
        y_vel = y_output * math.cos(heading) - x_output * math.sin(heading)

        target_vel = Twist()
        target_vel.linear.x = x_vel
        target_vel.linear.y = y_vel
        target_vel.angular.z = theta_output
        self.cmd_vel_publisher.publish(target_vel)
        self.get_logger().info(f"Publishing: {target_vel}")


def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()