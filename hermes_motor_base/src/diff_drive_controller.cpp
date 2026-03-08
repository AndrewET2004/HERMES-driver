#include "hermes_motor_base/diff_drive_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace hermes_driver
{

DiffDriveController::DiffDriveController(const rclcpp::NodeOptions & options)
: Node("diff_drive_controller", options)
{
  // ── Declare Parameters ──────────────────────────────────────────
  this->declare_parameter("wheel_separation", 0.30);   // metres
  this->declare_parameter("wheel_radius",     0.035);  // metres
  this->declare_parameter("max_rpm",          200.0);
  this->declare_parameter("pwm_frequency",    1000);

  // Stiction-compensation parameters
  this->declare_parameter("pwm_min_static",   0.15);   // minimum PWM during kickstart
  this->declare_parameter("pwm_kickstart_ms", 200.0);  // kickstart window [ms]
  this->declare_parameter("pwm_min_kinetic",  0.0);    // minimum PWM after kickstart

  // Default GPIO pin numbers (BCM numbering)
  this->declare_parameter("gpio_pwma", 12);
  this->declare_parameter("gpio_ain1", 23);
  this->declare_parameter("gpio_ain2", 24);
  this->declare_parameter("gpio_pwmb", 13);
  this->declare_parameter("gpio_bin1", 27);
  this->declare_parameter("gpio_bin2", 22);
  this->declare_parameter("gpio_stby", 25);
  this->declare_parameter("gpio_chip", std::string("gpiochip0"));

  // ── Read Parameters ─────────────────────────────────────────────
  wheel_separation_ = this->get_parameter("wheel_separation").as_double();
  wheel_radius_     = this->get_parameter("wheel_radius").as_double();

  double max_rpm    = this->get_parameter("max_rpm").as_double();
  max_motor_speed_  = max_rpm * 2.0 * M_PI / 60.0;  // convert RPM → rad/s

  int pwm_freq = this->get_parameter("pwm_frequency").as_int();

  pwm_min_static_   = this->get_parameter("pwm_min_static").as_double();
  pwm_kickstart_ms_ = this->get_parameter("pwm_kickstart_ms").as_double();
  pwm_min_kinetic_  = this->get_parameter("pwm_min_kinetic").as_double();

  // Or we can set pins based on input arguments (if they are given)
  TB6612Pins pins;
  pins.motor_a.pwm = this->get_parameter("gpio_pwma").as_int();
  pins.motor_a.in1 = this->get_parameter("gpio_ain1").as_int();
  pins.motor_a.in2 = this->get_parameter("gpio_ain2").as_int();
  pins.motor_b.pwm = this->get_parameter("gpio_pwmb").as_int();
  pins.motor_b.in1 = this->get_parameter("gpio_bin1").as_int();
  pins.motor_b.in2 = this->get_parameter("gpio_bin2").as_int();
  pins.stby        = this->get_parameter("gpio_stby").as_int();

  std::string chip_name = this->get_parameter("gpio_chip").as_string();

  // ── Initialise Hardware ─────────────────────────────────────────
  driver_ = std::make_unique<TB6612FNG>(pins, pwm_freq, chip_name);
  if (!driver_->init()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialise TB6612FNG driver");
    throw std::runtime_error("GPIO init failed");
  }
  RCLCPP_INFO(this->get_logger(), "TB6612FNG driver initialised (PWM %d Hz)", pwm_freq);

  // ── Subscribe to cmd_vel ────────────────────────────────────────
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "DiffDriveController ready  [sep=%.3f m, radius=%.4f m, max_rpm=%.0f, "
    "pwm_min_static=%.2f, pwm_kickstart_ms=%.0f, pwm_min_kinetic=%.2f]",
    wheel_separation_, wheel_radius_, max_rpm,
    pwm_min_static_, pwm_kickstart_ms_, pwm_min_kinetic_);
}

DiffDriveController::~DiffDriveController()
{
  if (driver_) {
    driver_->shutdown();
  }
}

void DiffDriveController::cmd_vel_callback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto [left, right] = twist_to_wheel_speeds(msg->linear.x, msg->angular.z);
  left  = apply_stiction_comp(left,  left_active_,  left_last_sign_,  left_start_time_);
  right = apply_stiction_comp(right, right_active_, right_last_sign_, right_start_time_);
  driver_->set_motors(left, right);
}

std::pair<double, double> DiffDriveController::twist_to_wheel_speeds(
  double linear, double angular) const
{
  // Differential drive inverse kinematics:
  //   v_left  = (linear - angular * wheel_separation / 2) / wheel_radius
  //   v_right = (linear + angular * wheel_separation / 2) / wheel_radius
  double v_left  = (linear - angular * wheel_separation_ / 2.0) / wheel_radius_;
  double v_right = (linear + angular * wheel_separation_ / 2.0) / wheel_radius_;

  // Normalise to [-1.0, 1.0] based on max motor speed
  double left_norm  = std::clamp(v_left  / max_motor_speed_, -1.0, 1.0);
  double right_norm = std::clamp(v_right / max_motor_speed_, -1.0, 1.0);

  return {left_norm, right_norm};
}

double DiffDriveController::apply_stiction_comp(
  double speed, bool & active, int & last_sign,
  std::chrono::steady_clock::time_point & start_time)
{
  // Determine the command sign: +1 forward, -1 reverse, 0 stopped.
  const int sign =
    (speed > kNearZeroThreshold) ? 1 : (speed < -kNearZeroThreshold) ? -1 : 0;

  if (sign == 0) {
    // Command is effectively zero: stop the motor and reset activation state so
    // the next non-zero command triggers a fresh kickstart.
    active    = false;
    last_sign = 0;
    return 0.0;
  }

  const double magnitude = std::abs(speed);

  if (!active || sign != last_sign) {
    // New activation or direction reversal: begin a fresh kickstart window.
    active     = true;
    last_sign  = sign;
    start_time = std::chrono::steady_clock::now();
    return sign * std::max(magnitude, pwm_min_static_);
  }

  // Check whether we are still inside the kickstart window.
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();

  if (elapsed_ms < pwm_kickstart_ms_) {
    // Still in kickstart phase: enforce the static floor.
    return sign * std::max(magnitude, pwm_min_static_);
  }

  // Post-kickstart: enforce only the kinetic floor (default 0.0), allowing
  // fine-grained speed control for accurate turns.
  return sign * std::max(magnitude, pwm_min_kinetic_);
}

}  // namespace hermes_driver

// ── main ────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hermes_driver::DiffDriveController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}