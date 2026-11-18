"""Pure-Python motor models (no ROS dependency, so they are easy to unit test)."""

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    """Limit value to the range [low, high]."""
    return max(low, min(high, value))


@dataclass
class ThrusterGeometry:
    """Maps body velocity (cmd_vel) to the target RPM of two thrusters."""

    thruster_separation: float = 0.30  # metres between left and right thruster
    max_linear_speed: float = 1.5      # m/s that corresponds to max_rpm
    max_rpm: float = 3000.0            # RPM limit of each motor

    def twist_to_rpm(self, linear_x: float, angular_z: float):
        """Differential-drive mixing: returns (left_rpm, right_rpm) targets."""
        v_left = linear_x - angular_z * self.thruster_separation / 2.0
        v_right = linear_x + angular_z * self.thruster_separation / 2.0
        left = v_left / self.max_linear_speed * self.max_rpm
        right = v_right / self.max_linear_speed * self.max_rpm
        return (
            clamp(left, -self.max_rpm, self.max_rpm),
            clamp(right, -self.max_rpm, self.max_rpm),
        )


class MotorSimulator:
    """First-order motor response with an acceleration (slew-rate) limit."""

    def __init__(self, time_constant: float = 0.4,
                 max_accel_rpm_s: float = 6000.0, max_rpm: float = 3000.0):
        self.time_constant = time_constant
        self.max_accel_rpm_s = max_accel_rpm_s
        self.max_rpm = max_rpm
        self.rpm = 0.0  # current simulated motor speed

    def step(self, target_rpm: float, dt: float) -> float:
        """Advance the motor by dt seconds towards target_rpm and return RPM."""
        if dt <= 0.0:
            return self.rpm
        if self.time_constant <= 0.0:
            desired = target_rpm  # ideal motor: responds instantly
        else:
            # Exact discretisation of  d(rpm)/dt = (target - rpm) / tau
            alpha = 1.0 - math.exp(-dt / self.time_constant)
            desired = self.rpm + (target_rpm - self.rpm) * alpha
        # Limit how fast the speed can change (motor/driver current limit)
        max_delta = self.max_accel_rpm_s * dt
        delta = clamp(desired - self.rpm, -max_delta, max_delta)
        self.rpm = clamp(self.rpm + delta, -self.max_rpm, self.max_rpm)
        return self.rpm
