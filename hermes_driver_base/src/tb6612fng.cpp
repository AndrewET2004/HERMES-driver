#include "hermes_driver_base/tb6612fng.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace hermes_driver
{

// ── Free helper: software-PWM loop run on its own thread ───────────────────
static void pwm_loop(
  std::atomic<unsigned> & duty, std::atomic<bool> & running,
  gpiod_line * line, int freq)
{
  const int period_us = 1000000 / freq;
  while (running.load(std::memory_order_relaxed)) {
    unsigned d = duty.load(std::memory_order_relaxed);

    if (d == 0) {
      gpiod_line_set_value(line, 0);
      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    } else if (d >= 255) {
      gpiod_line_set_value(line, 1);
      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    } else {
      int on_us  = (period_us * static_cast<int>(d)) / 255;
      int off_us = period_us - on_us;
      gpiod_line_set_value(line, 1);
      std::this_thread::sleep_for(std::chrono::microseconds(on_us));
      gpiod_line_set_value(line, 0);
      std::this_thread::sleep_for(std::chrono::microseconds(off_us));
    }
  }
}

// ── Constructor / Destructor ────────────────────────────────────────────────

TB6612FNG::TB6612FNG(
  const TB6612Pins & pins, int pwm_freq, const std::string & chip_name)
: pins_(pins), pwm_freq_(pwm_freq > 0 ? pwm_freq : 1000), chip_name_(chip_name)
{}

TB6612FNG::~TB6612FNG()
{
  shutdown();
}

// ── init ────────────────────────────────────────────────────────────────────

bool TB6612FNG::init()
{
  // Open the GPIO chip
  chip_ = gpiod_chip_open_by_name(chip_name_.c_str());
  if (!chip_) {
    std::fprintf(stderr, "[TB6612FNG] Failed to open GPIO chip '%s'\n", chip_name_.c_str());
    return false;
  }

  // Helper: claim one line as an output (initial value LOW)
  auto request_output = [&](int offset, const char * label) -> gpiod_line * {
    gpiod_line * line = gpiod_chip_get_line(chip_, static_cast<unsigned>(offset));
    if (!line) {
      std::fprintf(stderr, "[TB6612FNG] Failed to get GPIO line %d (%s)\n", offset, label);
      return nullptr;
    }
    if (gpiod_line_request_output(line, "hermes_driver", 0) < 0) {
      std::fprintf(
        stderr, "[TB6612FNG] Failed to request output on GPIO line %d (%s)\n",
        offset, label);
      return nullptr;
    }
    return line;
  };

  line_ain1_ = request_output(pins_.motor_a.in1, "AIN1");
  line_ain2_ = request_output(pins_.motor_a.in2, "AIN2");
  line_bin1_ = request_output(pins_.motor_b.in1, "BIN1");
  line_bin2_ = request_output(pins_.motor_b.in2, "BIN2");
  line_stby_ = request_output(pins_.stby,         "STBY");
  line_pwma_ = request_output(pins_.motor_a.pwm,  "PWMA");
  line_pwmb_ = request_output(pins_.motor_b.pwm,  "PWMB");

  if (!line_ain1_ || !line_ain2_ || !line_bin1_ || !line_bin2_ ||
      !line_stby_ || !line_pwma_ || !line_pwmb_)
  {
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }

  // Start software-PWM threads (one per motor channel)
  pwm_a_.line    = line_pwma_;
  pwm_a_.freq    = pwm_freq_;
  pwm_a_.duty    = 0;
  pwm_a_.running = true;
  pwm_a_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_a_.duty), std::ref(pwm_a_.running), line_pwma_, pwm_freq_);

  pwm_b_.line    = line_pwmb_;
  pwm_b_.freq    = pwm_freq_;
  pwm_b_.duty    = 0;
  pwm_b_.running = true;
  pwm_b_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_b_.duty), std::ref(pwm_b_.running), line_pwmb_, pwm_freq_);

  // Enable the driver (STBY HIGH)
  if (gpiod_line_set_value(line_stby_, 1) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to assert STBY pin %d\n", pins_.stby);
    // Stop threads and release the chip directly (initialised_ is not yet set,
    // so shutdown() would skip chip cleanup).
    pwm_a_.running = false;
    pwm_b_.running = false;
    if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
    if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }
    gpiod_chip_close(chip_);
    chip_ = nullptr;
    return false;
  }

  initialised_ = true;
  return true;
}

// ── set_motor (private) ─────────────────────────────────────────────────────

void TB6612FNG::set_motor(
  gpiod_line * in1, gpiod_line * in2, PwmState & pwm, double speed)
{
  if (!initialised_) {return;}

  speed = std::clamp(speed, -1.0, 1.0);

  // Direction logic:
  //   speed > 0  →  IN1 = HIGH, IN2 = LOW   (forward)
  //   speed < 0  →  IN1 = LOW,  IN2 = HIGH  (reverse)
  //   speed == 0 →  IN1 = LOW,  IN2 = LOW   (coast)
  int in1_val, in2_val;
  if (speed > 0.0) {
    in1_val = 1; in2_val = 0;
  } else if (speed < 0.0) {
    in1_val = 0; in2_val = 1;
  } else {
    in1_val = 0; in2_val = 0;
  }

  if (gpiod_line_set_value(in1, in1_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN1\n");
  }
  if (gpiod_line_set_value(in2, in2_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN2\n");
  }

  // Update duty cycle for the software-PWM thread (0–255)
  pwm.duty = std::min(255u, static_cast<unsigned>(std::abs(speed) * 255.0));
}

// ── Public motor setters ────────────────────────────────────────────────────

void TB6612FNG::set_motor_a(double speed)
{
  set_motor(line_ain1_, line_ain2_, pwm_a_, speed);
}

void TB6612FNG::set_motor_b(double speed)
{
  set_motor(line_bin1_, line_bin2_, pwm_b_, speed);
}

void TB6612FNG::set_motors(double speed_a, double speed_b)
{
  set_motor_a(speed_a);
  set_motor_b(speed_b);
}

void TB6612FNG::set_standby(bool enable)
{
  if (!initialised_) {return;}
  if (gpiod_line_set_value(line_stby_, enable ? 1 : 0) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write STBY pin %d\n", pins_.stby);
  }
}

// ── shutdown ────────────────────────────────────────────────────────────────

void TB6612FNG::shutdown()
{
  // Stop software-PWM threads first (safe even if never started)
  pwm_a_.running = false;
  pwm_b_.running = false;
  if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
  if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }

  if (!initialised_) {return;}

  // Drive all direction pins and STBY low before releasing the chip
  if (line_ain1_) {gpiod_line_set_value(line_ain1_, 0);}
  if (line_ain2_) {gpiod_line_set_value(line_ain2_, 0);}
  if (line_bin1_) {gpiod_line_set_value(line_bin1_, 0);}
  if (line_bin2_) {gpiod_line_set_value(line_bin2_, 0);}
  if (line_stby_) {gpiod_line_set_value(line_stby_, 0);}

  // gpiod_chip_close() releases all lines claimed from this chip
  gpiod_chip_close(chip_);
  chip_ = nullptr;
  initialised_ = false;
}

}  // namespace hermes_driver
