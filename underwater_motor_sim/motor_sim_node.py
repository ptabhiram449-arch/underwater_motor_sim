#!/usr/bin/env python3
"""ROS 2 node: cmd_vel -> target RPM -> simulated motor response -> telemetry."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray

from underwater_motor_sim.motor_model import MotorSimulator, ThrusterGeometry


class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')

        # ---- Parameters (overridable from YAML / command line) ----
        self.declare_parameter('thruster_separation', 0.30)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('time_constant', 0.4)
        self.declare_parameter('max_accel_rpm_s', 6000.0)
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)

        p = self.get_parameter
        max_rpm = p('max_rpm').value
        self.geometry = ThrusterGeometry(
            p('thruster_separation').value, p('max_linear_speed').value, max_rpm)
        self.left_motor = MotorSimulator(
            p('time_constant').value, p('max_accel_rpm_s').value, max_rpm)
        self.right_motor = MotorSimulator(
            p('time_constant').value, p('max_accel_rpm_s').value, max_rpm)
        self.cmd_timeout = p('cmd_timeout').value

        # ---- State ----
        self.target_left = 0.0
        self.target_right = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        # ---- ROS interfaces ----
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.left_pub = self.create_publisher(Float32, 'motor/left/rpm', 10)
        self.right_pub = self.create_publisher(Float32, 'motor/right/rpm', 10)
        self.target_pub = self.create_publisher(
            Float32MultiArray, 'motor/target_rpm', 10)

        period = 1.0 / p('update_rate_hz').value
        self.create_timer(period, self.update)
        self.get_logger().info('Motor simulator started (listening on cmd_vel)')

    def cmd_vel_callback(self, msg: Twist):
        """Convert an incoming velocity command to target motor RPM."""
        self.target_left, self.target_right = self.geometry.twist_to_rpm(
            msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """Timer: advance the motor simulation and publish telemetry."""
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        # Safety watchdog: stop the motors if commands stop arriving
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.target_left = 0.0
            self.target_right = 0.0

        left = self.left_motor.step(self.target_left, dt)
        right = self.right_motor.step(self.target_right, dt)

        self.left_pub.publish(Float32(data=float(left)))
        self.right_pub.publish(Float32(data=float(right)))
        self.target_pub.publish(Float32MultiArray(
            data=[float(self.target_left), float(self.target_right)]))


def main(args=None):
    rclpy.init(args=args)
    node = MotorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
