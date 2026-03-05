#include "hermes_driver_base/hermes_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hermes_driver
{

static constexpr char kHwLogger[] = "HermesHardware";

// ── on_init ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn HermesHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Validate joint count ────────────────────────────────────────────────
  if (info_.joints.size() != kNumWheels) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kHwLogger),
      "Expected exactly %zu joints (left + right wheel), got %zu",
      kNumWheels, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const auto & joint = info_.joints[i];
    joint_names_[i] = joint.name;

    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kHwLogger),
        "Joint '%s' must have exactly one velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kHwLogger),
        "Joint '%s' must have exactly two state interfaces (position + velocity)",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ── Read hardware parameters ────────────────────────────────────────────
  auto require_param = [&](const std::string & key) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      throw std::runtime_error(std::string("Missing hardware parameter: ") + key);
    }
    return it->second;
  };

  double max_rpm = std::stod(require_param("max_rpm"));
  if (max_rpm <= 0.0) {
    RCLCPP_FATAL(rclcpp::get_logger(kHwLogger), "max_rpm must be positive");
    return hardware_interface::CallbackReturn::ERROR;
  }
  max_motor_speed_ = max_rpm * 2.0 * M_PI / 60.0;  // RPM → rad/s

  int pwm_freq = std::stoi(require_param("pwm_frequency"));

  TB6612Pins pins;
  pins.motor_a.pwm = std::stoi(require_param("gpio_pwma"));
  pins.motor_a.in1 = std::stoi(require_param("gpio_ain1"));
  pins.motor_a.in2 = std::stoi(require_param("gpio_ain2"));
  pins.motor_b.pwm = std::stoi(require_param("gpio_pwmb"));
  pins.motor_b.in1 = std::stoi(require_param("gpio_bin1"));
  pins.motor_b.in2 = std::stoi(require_param("gpio_bin2"));
  pins.stby        = std::stoi(require_param("gpio_stby"));
  std::string chip_name = require_param("gpio_chip");

  driver_ = std::make_unique<TB6612FNG>(pins, pwm_freq, chip_name);

  // Reset state
  wheel_positions_.fill(0.0);
  wheel_velocities_.fill(0.0);
  wheel_commands_.fill(0.0);

  RCLCPP_INFO(
    rclcpp::get_logger(kHwLogger),
    "HermesHardware initialised  joints=[%s, %s]  max_motor_speed=%.2f rad/s",
    joint_names_[0].c_str(), joint_names_[1].c_str(), max_motor_speed_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── export_state_interfaces ────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
HermesHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &wheel_positions_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[i]);
  }
  return interfaces;
}

// ── export_command_interfaces ──────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
HermesHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &wheel_commands_[i]);
  }
  return interfaces;
}

// ── on_activate ────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn HermesHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!driver_->init()) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kHwLogger),
      "Failed to initialise TB6612FNG GPIO driver");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(kHwLogger), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ──────────────────────────────────────────────────────────

hardware_interface::CallbackReturn HermesHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (driver_) {
    driver_->shutdown();
  }
  RCLCPP_INFO(rclcpp::get_logger(kHwLogger), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── read ───────────────────────────────────────────────────────────────────

hardware_interface::return_type HermesHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No wheel encoders: integrate commanded velocity to estimate position.
  // The diff_drive_controller uses these states to publish odometry.
  const double dt = period.seconds();
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    wheel_velocities_[i] = wheel_commands_[i];
    wheel_positions_[i] += wheel_velocities_[i] * dt;
  }
  return hardware_interface::return_type::OK;
}

// ── write ──────────────────────────────────────────────────────────────────

hardware_interface::return_type HermesHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert rad/s commands to normalised speed [-1.0, 1.0] and drive motors.
  const double left_speed =
    std::clamp(wheel_commands_[0] / max_motor_speed_, -1.0, 1.0);
  const double right_speed =
    std::clamp(wheel_commands_[1] / max_motor_speed_, -1.0, 1.0);

  driver_->set_motors(left_speed, right_speed);
  return hardware_interface::return_type::OK;
}

}  // namespace hermes_driver

// ── Plugin registration ────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  hermes_driver::HermesHardware,
  hardware_interface::SystemInterface)
