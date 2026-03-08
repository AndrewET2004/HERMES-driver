#ifndef HERMES_MOTOR_BASE__DIFF_DRIVE_CONTROLLER_HPP_
#define HERMES_MOTOR_BASE__DIFF_DRIVE_CONTROLLER_HPP_

#include <chrono>
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
///   - pwm_min_static     [0..1]  minimum PWM during the kickstart phase (overcomes static friction)
///   - pwm_kickstart_ms   [ms]    duration of the kickstart window
///   - pwm_min_kinetic    [0..1]  minimum PWM after kickstart (0 = no floor; raise for rough terrain)
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

  /// Apply per-motor stiction compensation.
  ///
  /// Behaviour:
  ///   - A command at or below kNearZeroThreshold is treated as zero: the motor
  ///     is stopped and the activation state is reset so that the next non-zero
  ///     command triggers a fresh kickstart.
  ///   - On the first non-zero command after idle (or after a direction reversal),
  ///     a kickstart is started: pwm_min_static_ is enforced for pwm_kickstart_ms_
  ///     milliseconds to guarantee static friction is overcome.
  ///   - After the kickstart window expires, only pwm_min_kinetic_ is enforced
  ///     (default 0.0), giving the full commanded value for smooth, accurate turns.
  ///
  /// Note: the per-motor state passed by reference is only ever touched from
  /// cmd_vel_callback, which runs on the node's single-threaded executor
  /// (rclcpp::spin). No mutex is required.
  double apply_stiction_comp(
    double speed, bool & active, int & last_sign,
    std::chrono::steady_clock::time_point & start_time);

  // Parameters
  double wheel_separation_{0.0};
  double wheel_radius_{0.0};
  double max_motor_speed_{0.0};    // rad/s at duty 1.0, derived from max_rpm

  double pwm_min_static_{0.15};    // floor applied during the kickstart phase
  double pwm_min_kinetic_{0.0};    // floor applied after the kickstart phase
  double pwm_kickstart_ms_{200.0}; // kickstart window duration [ms]

  /// Commands whose absolute magnitude is at or below this value are treated as zero.
  static constexpr double kNearZeroThreshold = 0.01;

  // Per-motor activation state (for stiction compensation)
  bool   left_active_{false};
  bool   right_active_{false};
  int    left_last_sign_{0};
  int    right_last_sign_{0};
  std::chrono::steady_clock::time_point left_start_time_{};
  std::chrono::steady_clock::time_point right_start_time_{};

  // Hardware
  std::unique_ptr<TB6612FNG> driver_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace hermes_driver

#endif  // HERMES_MOTOR_BASE__DIFF_DRIVE_CONTROLLER_HPP_