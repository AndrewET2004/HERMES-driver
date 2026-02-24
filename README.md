# HERMES-driver
Low-level hardware driver for the HERMES robot platform.

Subscribes to `cmd_vel` (`geometry_msgs/msg/Twist`) and converts velocity commands into
PWM + direction signals for a **TB6612FNG** dual H-bridge motor driver connected via GPIO.

## Hardware Wiring (TB6612FNG)

| TB6612FNG Pin | Function         | GPIO Pin (example) |
|---------------|------------------|--------------------|
| PWMA          | Motor A speed    | configurable       |
| AIN1          | Motor A dir 1    | configurable       |
| AIN2          | Motor A dir 2    | configurable       |
| PWMB          | Motor B speed    | configurable       |
| BIN1          | Motor B dir 1    | configurable       |
| BIN2          | Motor B dir 2    | configurable       |
| STBY          | Standby (HIGH=on)| configurable       |

## Dependencies

- **libgpiod v2** — Linux GPIO character device library (v2.x API)  
  ```bash
  sudo apt install libgpiod-dev   # Raspberry Pi OS Bookworm ships v2
  ```

## Building
```bash
cd ~/hermes_ws
colcon build --packages-select hermes_driver_base
source install/setup.bash
```

## Running
```bash
ros2 launch hermes_driver_base driver.launch.py
```

## Parameters
See `config/driver_params.yaml` for all tunable parameters (GPIO pins, max RPM, wheel
separation, wheel radius, PWM frequency, etc.).