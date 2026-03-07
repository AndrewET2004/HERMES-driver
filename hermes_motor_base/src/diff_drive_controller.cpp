#include "hermes_motor_base/diff_drive_controller.hpp"

#include <algorithm>
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

  // Minimum duty-cycle fraction for non-zero motor commands.
  // Ensures the motor always receives enough power to overcome static friction
  // even when cmd_vel sends very small velocities.
  this->declare_parameter("min_pwm_fraction",  0.15);

  // Normalised speed below which the motor command is treated as zero.
  // Prevents navigation micro-corrections from snapping the motor to the
  // minimum-PWM floor and causing oscillation / overshoot.
  this->declare_parameter("velocity_deadband", 0.05);

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

  int    pwm_freq          = this->get_parameter("pwm_frequency").as_int();
  double min_pwm_fraction  = this->get_parameter("min_pwm_fraction").as_double();
  double velocity_deadband = this->get_parameter("velocity_deadband").as_double();

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
  driver_ = std::make_unique<TB6612FNG>(
    pins, pwm_freq, chip_name, min_pwm_fraction, velocity_deadband);
  if (!driver_->init()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to initialise TB6612FNG driver");
    throw std::runtime_error("GPIO init failed");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "TB6612FNG driver initialised  PWM %d Hz  min_pwm=%.0f%%  deadband=%.0f%%",
    pwm_freq, min_pwm_fraction * 100.0, velocity_deadband * 100.0);

  // ── Subscribe to cmd_vel ────────────────────────────────────────
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&DiffDriveController::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "DiffDriveController ready  [sep=%.3f m, radius=%.4f m, max_rpm=%.0f]",
    wheel_separation_, wheel_radius_, max_rpm);
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