#ifndef HERMES_MOTOR_BASE__TB6612FNG_HPP_
#define HERMES_MOTOR_BASE__TB6612FNG_HPP_

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
///   - Initialise GPIO pins (via libgpiod v2)
///   - Accept a speed value [-1.0, 1.0] per motor and translate to
///     PWM duty-cycle + direction pin states using software PWM threads
///   - Enable / disable the driver via the STBY pin
class TB6612FNG
{
public:
  /// @param pins        GPIO pin mapping
  /// @param pwm_freq    PWM frequency in Hz (typical: 100–2000)
  /// @param chip_name   gpiochip device name or path (e.g. "gpiochip0" or
  ///                    "/dev/gpiochip0")
  explicit TB6612FNG(
    const TB6612Pins & pins,
    int pwm_freq = 1000,
    const std::string & chip_name = "gpiochip0");
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
  /// The PWM thread owns exclusive access to its request; no locking needed.
  struct PwmState
  {
    std::atomic<unsigned>  duty{0};       // 0 (off) … 255 (full on)
    std::atomic<bool>      running{false};
    gpiod_line_request *   request{nullptr};  // single-line request (PWM pin)
    unsigned int           offset{0};         // GPIO line offset
    int                    freq{1000};
    std::thread            thread;
  };

  void set_motor(unsigned int in1_off, unsigned int in2_off, PwmState & pwm, double speed);

  TB6612Pins  pins_;
  int         pwm_freq_;
  std::string chip_name_;
  bool        initialised_{false};

  // libgpiod v2 handles
  gpiod_chip *         chip_{nullptr};
  /// Request for the five direction/standby lines (ain1, ain2, bin1, bin2, stby).
  /// Only ever accessed from the main thread.
  gpiod_line_request * dir_request_{nullptr};

  // Software PWM channels (one per motor); each owns a single-line request.
  PwmState pwm_a_;
  PwmState pwm_b_;
};

}  // namespace hermes_driver

#endif  // HERMES_MOTOR_BASE__TB6612FNG_HPP_
