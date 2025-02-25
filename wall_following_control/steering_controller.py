#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import math
from pynput import keyboard


class SteeringNode(Node):
    def __init__(self):
        super().__init__("steering_node")
        self.get_logger().info("the steering node has started")
        scan_topic_param = self.declare_parameter('scan_topic', '/scan')
        drive_topic_param =  self.declare_parameter('drive_topic', '/drive')
        self.goal_distance_param = self.declare_parameter('goal_distance', 0.0)
        self.speed_param = self.declare_parameter('speed', 0.0)
        self.boost_param = self.declare_parameter('boost', 0.0)
        self.kp_param = self.declare_parameter('kp', 0.0)
        self.kd_param = self.declare_parameter('kd', 0.0)
        self.ki_param = self.declare_parameter('ki', 0.0)
        self.max_steering_param = self.declare_parameter('max_steering', 0.0)
        self.reference_angle_param = self.declare_parameter('reference_angle', 0.0)
        self.sub_scan = self.create_subscription(LaserScan, scan_topic_param.get_parameter_value().string_value, self.scan_cb, 10)
        self.pub_vel_cmd = self.create_publisher(AckermannDriveStamped, drive_topic_param.get_parameter_value().string_value, 10)
        self.tmr = self.create_timer(0.01, self.steering_pid_controller)
        self.scan_msg = None
        self.goal_distance = self.goal_distance_param.get_parameter_value().double_value
        self.right_distance = 0.0
        self.prev_error = 0.0
        self.vel_cmd = AckermannDriveStamped()
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

    def on_press(self, key):
        try:
            if key.char == 'b':
                self.vel_cmd.drive.speed = self.boost_param.get_parameter_value().double_value  # boost forward
            if key.char == 'w':
                self.vel_cmd.drive.speed = self.speed_param.get_parameter_value().double_value  # Move forward
            if key.char == 's':
                self.vel_cmd.drive.speed = -0.5  # Move backwards

        except AttributeError:
            self.get_logger().warn("error while sending.. :(")

    def on_release(self, key):
        # Stop the robot when the key is released
        self.vel_cmd.drive.speed = 0.0
        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def scan_cb(self, msg:LaserScan):
        reference_angle = math.radians(self.reference_angle_param.get_parameter_value().double_value)
        angle_index = int((reference_angle - msg.angle_min) / msg.angle_increment)
        if 0 <= angle_index < len(msg.ranges):
            self.right_distance = msg.ranges[angle_index]
        else:
            self.get_logger().info(f"error")

    def steering_pid_controller(self):
        error = (self.goal_distance - self.right_distance)
        kp = self.kp_param.get_parameter_value().double_value
        kd = self.kd_param.get_parameter_value().double_value
        p_controller = kp * error
        d_controller = kd * (error - self.prev_error)
        self.prev_error = error
        steering_angle = p_controller + d_controller
        max_steering_angle = self.max_steering_param.get_parameter_value().double_value

        # capping the steering angle
        if abs(steering_angle) > max_steering_angle:
            if steering_angle < 0:
                steering_angle = -max_steering_angle
            else:
                steering_angle = max_steering_angle
        self.vel_cmd.drive.steering_angle = steering_angle
        
        #publishing the velocity command
        self.pub_vel_cmd.publish(self.vel_cmd)
        self.get_logger().info(f"Distance to the right wall {self.right_distance} steering: {self.vel_cmd.drive.steering_angle}")


def main():
    rclpy.init()
    node = SteeringNode()
    rclpy.spin(node=node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    