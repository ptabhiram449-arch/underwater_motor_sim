import pytest

from underwater_motor_sim.motor_model import MotorSimulator, ThrusterGeometry


def test_straight_ahead_equal_rpm():
    left, right = ThrusterGeometry().twist_to_rpm(0.75, 0.0)
    assert left == pytest.approx(1500.0)
    assert right == pytest.approx(1500.0)


def test_turn_in_place_opposite_rpm():
    left, right = ThrusterGeometry().twist_to_rpm(0.0, 2.0)
    assert left == pytest.approx(-right)
    assert right > 0.0


def test_rpm_is_clamped():
    left, right = ThrusterGeometry().twist_to_rpm(100.0, 0.0)
    assert left == 3000.0 and right == 3000.0


def test_motor_converges_to_target():
    motor = MotorSimulator(time_constant=0.2, max_accel_rpm_s=1e9)
    for _ in range(500):
        rpm = motor.step(1000.0, 0.01)
    assert rpm == pytest.approx(1000.0, abs=1.0)


def test_motor_first_order_63_percent_at_tau():
    motor = MotorSimulator(time_constant=0.5, max_accel_rpm_s=1e9)
    for _ in range(50):
        rpm = motor.step(1000.0, 0.01)  # 0.5 s == one time constant
    assert rpm == pytest.approx(632.1, abs=1.0)


def test_acceleration_limit():
    motor = MotorSimulator(time_constant=0.0, max_accel_rpm_s=1000.0)
    assert motor.step(3000.0, 0.1) == pytest.approx(100.0)


def test_motor_never_exceeds_max_rpm():
    motor = MotorSimulator(time_constant=0.0, max_accel_rpm_s=1e9, max_rpm=3000.0)
    assert motor.step(9999.0, 1.0) == 3000.0
