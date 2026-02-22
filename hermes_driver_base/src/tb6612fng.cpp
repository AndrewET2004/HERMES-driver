#include "hermes_driver_base/tb6612fng.hpp"

#include <lgpio.h>   

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace hermes_driver
{

TB6612FNG::TB6612FNG(const TB6612Pins & pins, int pwm_freq)
: pins_(pins), pwm_freq_(pwm_freq)
{}

TB6612FNG::~TB6612FNG()
{
  shutdown();
}

bool TB6612FNG::init()
{
  // Open the GPIO chip (4 for rpi5 i think)
  gpio_handle_ = lgGpiochipOpen(4);
  if (gpio_handle_ < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to open GPIO chip 0\n");
    return false;
  }

  // Claim direction pins as outputs (initially LOW)
  auto claim_out = [&](int pin) {
    return lgGpioClaimOutput(gpio_handle_, 0, pin, 0) == 0;
  };

  if (!claim_out(pins_.motor_a.in1) || !claim_out(pins_.motor_a.in2) ||
      !claim_out(pins_.motor_b.in1) || !claim_out(pins_.motor_b.in2) ||
      !claim_out(pins_.stby))
  {
    std::fprintf(stderr, "[TB6612FNG] Failed to claim direction/stby pins\n");
    return false;
  }

  // Enable the driver
  lgGpioWrite(gpio_handle_, pins_.stby, 1);

  initialised_ = true;
  return true;
}

void TB6612FNG::set_motor(const MotorPins & mp, double speed)
{
  if (!initialised_) {return;}

  speed = std::clamp(speed, -1.0, 1.0);

  // Direction logic:
  //   speed > 0  →  IN1 = HIGH, IN2 = LOW   (forward)
  //   speed < 0  →  IN1 = LOW,  IN2 = HIGH  (reverse)
  //   speed == 0 →  IN1 = LOW,  IN2 = LOW   (coast / brake)
  if (speed > 0.0) {
    lgGpioWrite(gpio_handle_, mp.in1, 1);
    lgGpioWrite(gpio_handle_, mp.in2, 0);
  } else if (speed < 0.0) {
    lgGpioWrite(gpio_handle_, mp.in1, 0);
    lgGpioWrite(gpio_handle_, mp.in2, 1);
  } else {
    lgGpioWrite(gpio_handle_, mp.in1, 0);
    lgGpioWrite(gpio_handle_, mp.in2, 0);
  }

  // Magnitude setting with duty cycle: 0–100 %  (fp)
  double duty = std::abs(speed) * 100.0;
  lgTxPwm(gpio_handle_, mp.pwm, pwm_freq_, duty, 0, 0);
}

void TB6612FNG::set_motor_a(double speed) { set_motor(pins_.motor_a, speed); }
void TB6612FNG::set_motor_b(double speed) { set_motor(pins_.motor_b, speed); }

// Assumes motor a controls left wheel and motor b controls right wheel
void TB6612FNG::set_motors(double speed_a, double speed_b)
{
  set_motor_a(speed_a);
  set_motor_b(speed_b);
}

void TB6612FNG::set_standby(bool enable)
{
  if (!initialised_) {return;}
  lgGpioWrite(gpio_handle_, pins_.stby, enable ? 1 : 0);
}

void TB6612FNG::shutdown()
{
  if (!initialised_) {return;}

  // Stop both motors
  set_motors(0.0, 0.0);
  set_standby(false);

  // Release the GPIO chip
  lgGpiochipClose(gpio_handle_);
  initialised_ = false;
}

}  // namespace hermes_driver