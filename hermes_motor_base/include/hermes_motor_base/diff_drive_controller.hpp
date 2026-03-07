#ifndef HERMES_DRIVER_BASE__DIFF_DRIVE_CONTROLLER_HPP_
#define HERMES_DRIVER_BASE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "hermes_motor_base/tb6612fng.hpp"

namespace hermes_driver
{

/// ROS 2 node that converts Twist (cmd_vel) → differential-drive motor commands.
///
/// Subscribes to:
///   /cmd_vel  (geometry_msgs/msg/Twist)
///
/// Parameters (all declared & configurable via YAML):
///   - wheel_separation   [m]   distance between left & right wheel contact patches
///   - wheel_radius       [m]   radius of the drive wheels
///   - max_rpm            [double]  maximum motor RPM at duty-cycle 1.0
///   - pwm_frequency      [Hz]
///   - gpio_pwma, gpio_ain1, gpio_ain2   Motor A GPIO BCM pin numbers
///   - gpio_pwmb, gpio_bin1, gpio_bin2   Motor B GPIO BCM pin numbers
///   - gpio_stby                         Standby GPIO BCM pin number
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DiffDriveController() override;

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Convert linear + angular velocity to per-wheel speed [-1.0, 1.0].
  std::pair<double, double> twist_to_wheel_speeds(double linear, double angular) const;

  // Parameters
  double wheel_separation_{0.0};
  double wheel_radius_{0.0};
  double max_motor_speed_{0.0};   // rad/s at duty 1.0, derived from max_rpm

  // Hardware
  std::unique_ptr<TB6612FNG> driver_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace hermes_driver

#endif  // HERMES_DRIVER_BASE__DIFF_DRIVE_CONTROLLER_HPP_