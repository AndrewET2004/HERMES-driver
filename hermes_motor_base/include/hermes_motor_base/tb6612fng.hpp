#ifndef HERMES_DRIVER_BASE__TB6612FNG_HPP_
#define HERMES_DRIVER_BASE__TB6612FNG_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include <gpiod.h>

namespace hermes_driver
{

/// Pin configuration for a single motor channel on the TB6612FNG.
struct MotorPins
{
  int pwm;   // PWM pin (speed control)
  int in1;   // Direction input 1
  int in2;   // Direction input 2
};

/// Pin configuration for the entire TB6612FNG breakout.
struct TB6612Pins
{
  MotorPins motor_a;  // Left motor  (AIN1, AIN2, PWMA)
  MotorPins motor_b;  // Right motor (BIN1, BIN2, PWMB)
  int stby;           // Standby pin (HIGH = enabled)
};

/// Low-level driver for the TB6612FNG dual H-bridge motor driver.
///
/// Responsibilities:
///   - Initialise GPIO pins (via libgpiod v1)
///   - Accept a speed value [-1.0, 1.0] per motor and translate to
///     PWM duty-cycle + direction pin states using software PWM threads
///   - Enable / disable the driver via the STBY pin
///
/// Min-PWM remapping and velocity deadband
/// ----------------------------------------
/// DC motors have a static-friction threshold below which they stall.
/// @p min_pwm_fraction (0–1) sets the minimum duty cycle applied whenever a
/// non-zero speed command is issued.  The raw speed is remapped:
///
///   duty = min_pwm + (1 – min_pwm) * |speed|
///
/// so that both small and large commands preserve their proportional
/// relationship while always overcoming the stall threshold.
///
/// @p velocity_deadband (0–1) is a normalised-speed threshold below which the
/// motor is unconditionally stopped.  This prevents navigation micro-
/// corrections (which would otherwise each snap the motor to min_pwm) from
/// causing oscillation / overshoot.
class TB6612FNG
{
public:
  /// @param pins               GPIO pin mapping
  /// @param pwm_freq           PWM frequency in Hz (typical: 100–2000)
  /// @param chip_name          gpiochip device name or path (e.g. "gpiochip0")
  /// @param min_pwm_fraction   Minimum duty-cycle fraction [0, 1] for any
  ///                           non-zero speed command (default 0 = no floor)
  /// @param velocity_deadband  Normalised speed [0, 1] below which the motor
  ///                           is held at zero (default 0 = no deadband)
  explicit TB6612FNG(
    const TB6612Pins & pins,
    int pwm_freq = 1000,
    const std::string & chip_name = "gpiochip0",
    double min_pwm_fraction = 0.0,
    double velocity_deadband = 0.0);
  ~TB6612FNG();

  // Prevent copies (owns GPIO handles)
  TB6612FNG(const TB6612FNG &) = delete;
  TB6612FNG & operator=(const TB6612FNG &) = delete;

  /// Initialise the GPIO chip and claim all pins. Returns true on success.
  bool init();

  /// Set motor A speed.  @param speed  [-1.0 .. 1.0]  (negative = reverse)
  void set_motor_a(double speed);

  /// Set motor B speed.  @param speed  [-1.0 .. 1.0]  (negative = reverse)
  void set_motor_b(double speed);

  /// Convenience: set both motors at once.
  void set_motors(double speed_a, double speed_b);

  /// Pull STBY HIGH (enable) or LOW (coast to stop).
  void set_standby(bool enable);

  /// Coast both motors to stop and release GPIO resources.
  void shutdown();

private:
  /// Software-PWM state for one motor channel.
  /// The PWM thread owns exclusive access to its line; no locking needed.
  struct PwmState
  {
    std::atomic<unsigned>  duty{0};       // 0 (off) … 255 (full on)
    std::atomic<bool>      running{false};
    gpiod_line *           line{nullptr};  // single-line handle (PWM pin)
    int                    freq{1000};
    std::thread            thread;
  };

  void set_motor(gpiod_line * in1_line, gpiod_line * in2_line, PwmState & pwm, double speed);

  TB6612Pins  pins_;
  int         pwm_freq_;
  std::string chip_name_;
  double      min_pwm_fraction_;   // minimum duty-cycle fraction for non-zero commands
  double      velocity_deadband_;  // normalised speed below which motor is stopped
  bool        initialised_{false};

  // libgpiod v1 handles
  gpiod_chip * chip_{nullptr};

  /// Individual direction and standby line handles.
  /// Only ever accessed from the main thread.
  gpiod_line * line_in1_a_{nullptr};
  gpiod_line * line_in2_a_{nullptr};
  gpiod_line * line_in1_b_{nullptr};
  gpiod_line * line_in2_b_{nullptr};
  gpiod_line * line_stby_{nullptr};

  // Software PWM channels (one per motor); each owns a single-line handle.
  PwmState pwm_a_;
  PwmState pwm_b_;
};

}  // namespace hermes_driver

#endif  // HERMES_DRIVER_BASE__TB6612FNG_HPP_
