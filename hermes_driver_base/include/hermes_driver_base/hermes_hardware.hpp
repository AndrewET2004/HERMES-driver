#ifndef HERMES_DRIVER_BASE__HERMES_HARDWARE_HPP_
#define HERMES_DRIVER_BASE__HERMES_HARDWARE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hermes_driver_base/tb6612fng.hpp"

namespace hermes_driver
{

/// ros2_control SystemInterface plugin for the HERMES differential-drive robot.
///
/// Maps ros2_control joint velocity commands → TB6612FNG PWM/direction signals.
/// Since the robot has no wheel encoders the read() method integrates the
/// commanded velocity to produce an open-loop odometry estimate.
///
/// Hardware parameters (set inside the <ros2_control> URDF block):
///   max_rpm          – free-spin RPM of the DC motors
///   pwm_frequency    – software PWM frequency in Hz
///   gpio_chip        – gpiochip device name (e.g. "gpiochip0")
///   gpio_pwma/ain1/ain2 – Motor A GPIO BCM pins
///   gpio_pwmb/bin1/bin2 – Motor B GPIO BCM pins
///   gpio_stby        – Standby GPIO BCM pin
///
/// Joints (in URDF <ros2_control> order, index 0 = left, 1 = right):
///   command_interface: velocity  (rad/s)
///   state_interfaces:  position (rad), velocity (rad/s)
class HermesHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HermesHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Integrate commanded velocities into estimated wheel positions.
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /// Convert velocity commands (rad/s) to normalised motor speed [-1, 1]
  /// and send them to the TB6612FNG hardware driver.
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kNumWheels = 2;

  // Low-level TB6612FNG GPIO driver
  std::unique_ptr<TB6612FNG> driver_;

  // Maximum wheel angular speed [rad/s], derived from max_rpm parameter
  double max_motor_speed_{0.0};

  // Per-wheel state and command arrays (index 0 = left, 1 = right)
  std::array<double, kNumWheels> wheel_positions_{0.0, 0.0};
  std::array<double, kNumWheels> wheel_velocities_{0.0, 0.0};
  std::array<double, kNumWheels> wheel_commands_{0.0, 0.0};

  // Joint names read from URDF hardware info (same order as above)
  std::array<std::string, kNumWheels> joint_names_;
};

}  // namespace hermes_driver

#endif  // HERMES_DRIVER_BASE__HERMES_HARDWARE_HPP_
