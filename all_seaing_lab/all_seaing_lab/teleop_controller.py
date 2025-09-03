#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopController(Node):
    def __init__(self):
        super().__init__("teleop_controller")

        self.declare_parameter("joy_x_scale", 2.0)
        self.declare_parameter("joy_y_scale", -1.0)
        self.declare_parameter("joy_ang_scale", -0.8)

        self.joy_x_scale = self.get_parameter("joy_x_scale").value
        self.joy_y_scale = self.get_parameter("joy_y_scale").value
        self.joy_ang_scale = self.get_parameter("joy_ang_scale").value

        self.enter_held = False
        self.in_teleop = True

        self.cmd_vel_pub = self.create_publisher(
            Twist, "cmd_vel", 10
        )
        self.joy_control_sub = self.create_subscription(
            Joy, "/joy", self.keyboard_callback, 10
        )

        self.get_logger().info("Starting onshore node, teleop enabled")

    def send_controls(self, x, y, angular):
        target_vel = Twist()
        target_vel.linear.x = x
        target_vel.linear.y = y
        target_vel.angular.z = angular
        self.cmd_vel_pub.publish(target_vel)

    def keyboard_callback(self, msg):
        # if msg.buttons[0]:  # msg.buttons[0] = space bar --> e-stop (reference config)

        if msg.buttons[1]:  # msg.buttons[1] = return key (reference config)
            if not self.enter_held:
                self.enter_held = True
                self.in_teleop = not self.in_teleop
                self.get_logger().info(f"Toggled teleop (now {self.in_teleop})")
        elif self.enter_held:
            self.enter_held = False

        if self.in_teleop:
            self.send_controls(
                msg.axes[1] * self.joy_x_scale,
                msg.axes[0] * self.joy_y_scale,
                msg.axes[2] * self.joy_ang_scale,
            )


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()