# underwater_motor_sim

ROS 2 (Python, `ament_python`) package that simulates basic motor firmware for an
underwater robot with two thrusters. It subscribes to `cmd_vel`, converts the
command into target motor RPM, simulates the motor's physical response and
publishes RPM telemetry.

## Pipeline

```
/cmd_vel (Twist) -> differential mixing -> target RPM
                 -> first-order lag + acceleration limit (simulated motor)
                 -> telemetry topics
```

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | subscribe | `linear.x` (m/s), `angular.z` (rad/s) |
| `motor/left/rpm` | `std_msgs/Float32` | publish | simulated left motor RPM |
| `motor/right/rpm` | `std_msgs/Float32` | publish | simulated right motor RPM |
| `motor/target_rpm` | `std_msgs/Float32MultiArray` | publish | `[left_target, right_target]` |

## Parameters (`config/params.yaml`)

| Name | Default | Meaning |
|---|---|---|
| `thruster_separation` | 0.30 | m between thrusters |
| `max_linear_speed` | 1.5 | m/s that maps to `max_rpm` |
| `max_rpm` | 3000.0 | RPM limit |
| `time_constant` | 0.4 | s, first-order response |
| `max_accel_rpm_s` | 6000.0 | RPM/s acceleration limit |
| `update_rate_hz` | 50.0 | simulation + telemetry rate |
| `cmd_timeout` | 0.5 | s, motors stop if no command arrives |

## Build and run

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <https://github.com/ptabhiram449-arch/underwater_motor_sim.git> underwater_motor_sim
cd ~/ros2_ws
colcon build --packages-select underwater_motor_sim
source install/setup.bash
ros2 launch underwater_motor_sim motor_sim.launch.py
```

In a second terminal (remember to `source install/setup.bash` too):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
ros2 topic echo /motor/left/rpm
ros2 topic echo /motor/right/rpm
```

Stop publishing and the RPM decays to zero (watchdog + motor response).

## Tests

```bash
colcon test --packages-select underwater_motor_sim && colcon test-result --verbose
# or, without ROS: python3 -m pytest test
```

## Design notes

- `motor_model.py` is pure Python (no ROS imports) so it can be unit tested alone.
- Motor response uses the exact discretisation of a first-order system,
  so the result does not depend on the timer rate.
- A command watchdog zeros the target when `cmd_vel` goes stale (fail-safe).
