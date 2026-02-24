#include "hermes_driver_base/tb6612fng.hpp"

#include <pigpio.h>

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
  // Initialise the pigpio library
  if (gpioInitialise() < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to initialise pigpio\n");
    return false;
  }

  // Helper: configure a pin as output, drive it LOW, and report any error
  auto setup_pin = [&](int pin) -> bool {
    if (gpioSetMode(pin, PI_OUTPUT) != 0) {
      std::fprintf(stderr, "[TB6612FNG] Failed to set mode for GPIO pin %d\n", pin);
      return false;
    }
    if (gpioWrite(pin, 0) != 0) {
      std::fprintf(stderr, "[TB6612FNG] Failed to write LOW to GPIO pin %d\n", pin);
      return false;
    }
    return true;
  };

  if (!setup_pin(pins_.motor_a.in1) || !setup_pin(pins_.motor_a.in2) ||
      !setup_pin(pins_.motor_b.in1) || !setup_pin(pins_.motor_b.in2) ||
      !setup_pin(pins_.stby))
  {
    gpioTerminate();
    return false;
  }

  // Set PWM frequency for both motor PWM pins
  if (gpioSetPWMfrequency(pins_.motor_a.pwm, pwm_freq_) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to set PWM frequency for motor A pin %d\n",
      pins_.motor_a.pwm);
    gpioTerminate();
    return false;
  }
  if (gpioSetPWMfrequency(pins_.motor_b.pwm, pwm_freq_) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to set PWM frequency for motor B pin %d\n",
      pins_.motor_b.pwm);
    gpioTerminate();
    return false;
  }

  // Enable the driver (STBY HIGH)
  if (gpioWrite(pins_.stby, 1) != 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to assert STBY pin %d\n", pins_.stby);
    gpioTerminate();
    return false;
  }

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
  int in1_val, in2_val;
  if (speed > 0.0) {
    in1_val = 1;
    in2_val = 0;
  } else if (speed < 0.0) {
    in1_val = 0;
    in2_val = 1;
  } else {
    in1_val = 0;
    in2_val = 0;
  }

  if (gpioWrite(mp.in1, in1_val) != 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN1 on GPIO pin %d\n", mp.in1);
  }
  if (gpioWrite(mp.in2, in2_val) != 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN2 on GPIO pin %d\n", mp.in2);
  }

  // pigpio gpioPWM duty cycle range: 0–255
  unsigned duty = std::min(255u, static_cast<unsigned>(std::abs(speed) * 255.0));
  if (gpioPWM(mp.pwm, duty) != 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to set PWM duty on GPIO pin %d\n", mp.pwm);
  }
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
  if (gpioWrite(pins_.stby, enable ? 1 : 0) != 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write STBY pin %d\n", pins_.stby);
  }
}

void TB6612FNG::shutdown()
{
  if (!initialised_) {return;}

  // Stop both motors
  set_motors(0.0, 0.0);
  set_standby(false);

  // Release pigpio resources
  gpioTerminate();
  initialised_ = false;
}

}  // namespace hermes_driver