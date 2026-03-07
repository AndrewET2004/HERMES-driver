# HERMES-driver
Low-level hardware driver for the HERMES differential-drive robot platform.

Uses the **ros2_control** framework: a `HermesHardware` plugin (implementing
`hardware_interface::SystemInterface`) wraps the TB6612FNG GPIO driver, and the
standard `diff_drive_controller` handles `/cmd_vel` subscribing and `/odom` publishing.

## Architecture

```
/cmd_vel (Twist)
    │
    ▼
diff_drive_controller          ← ros2_controllers standard plugin
    │  velocity commands (rad/s)
    ▼
HermesHardware plugin          ← hermes_motor_base/HermesHardware
    │  normalised speed [-1, 1]
    ▼
TB6612FNG GPIO driver          ← libgpiod v2
    │  PWM + direction pins
    ▼
DC motors (left + right)

diff_drive_controller also publishes:
  /odom (nav_msgs/msg/Odometry)  – open-loop, integrated from commanded velocities
  /tf   odom → base_link
```

## Hardware Wiring (TB6612FNG)

| TB6612FNG Pin | Function          | Default GPIO (BCM) |
|---------------|-------------------|--------------------|
| PWMA          | Motor A speed     | 12 (configurable)  |
| AIN1          | Motor A dir 1     | 23                 |
| AIN2          | Motor A dir 2     | 24                 |
| PWMB          | Motor B speed     | 13 (configurable)  |
| BIN1          | Motor B dir 1     | 27                 |
| BIN2          | Motor B dir 2     | 22                 |
| STBY          | Standby (HIGH=on) | 25                 |

All GPIO pins, wheel geometry, and motor limits are configured in
`urdf/hermes.urdf.xacro` (inside the `<ros2_control>` block) and in
`config/ros2_controllers.yaml`.

## Dependencies

- **libgpiod v2** — Linux GPIO character device library
  ```bash
  sudo apt install libgpiod-dev   # Raspberry Pi OS Bookworm ships v2
  ```
- **ros2_control** — hardware_interface + controller_manager
- **ros2_controllers** — diff_drive_controller, joint_state_broadcaster

## Building

```bash
cd ~/hermes_ws
colcon build --packages-select hermes_motor_base
source install/setup.bash
```

## Running (ros2_control approach — recommended)

```bash
ros2 launch hermes_motor_base hermes_control.launch.py
```

This starts:
1. `robot_state_publisher` — publishes the robot URDF/TF tree
2. `controller_manager` — loads the `HermesHardware` plugin
3. `joint_state_broadcaster` — re-publishes wheel states on `/joint_states`
4. `diff_drive_controller` — listens on `/cmd_vel`, publishes `/odom` and `/tf`

### Legacy standalone node (backward-compatible)

```bash
ros2 launch hermes_motor_base driver.launch.py
```

The original single-node approach is preserved for simple deployments that do
not need odometry. It reads `/cmd_vel` directly and drives the motors.

## Parameters

| File | Purpose |
|------|---------|
| `urdf/hermes.urdf.xacro` | Robot model + `<ros2_control>` hardware params (GPIO pins, max RPM, PWM freq) |
| `config/ros2_controllers.yaml` | diff_drive_controller params (wheel geometry, velocity limits, odometry settings) |
| `config/driver_params.yaml` | Legacy standalone-node params (unchanged) |
