#include "hermes_motor_base/tb6612fng.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace hermes_driver
{

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Convert a chip name like "gpiochip0" to a full device path "/dev/gpiochip0".
/// If the argument already starts with '/' it is returned unchanged.
static std::string to_chip_path(const std::string & name)
{
  if (name.empty() || name[0] == '/') {
    return name;
  }
  return "/dev/" + name;
}

/// Create a single-line output request on @p chip for @p offset.
/// Returns nullptr on failure.
/// Note: in libgpiod v1 the gpiod_line handle is owned by the chip and must
/// NOT be freed independently.  Only the request (acquired by
/// gpiod_line_request_output) needs to be released via gpiod_line_release().
/// If gpiod_line_request_output fails there is no request to release, so no
/// cleanup is required beyond closing the chip.
static gpiod_line * request_output_line(
  gpiod_chip * chip, unsigned int offset, const char * consumer)
{
  gpiod_line * line = gpiod_chip_get_line(chip, offset);
  if (!line) {return nullptr;}
  if (gpiod_line_request_output(line, consumer, 0) < 0) {return nullptr;}
  return line;
}

// ── Software-PWM loop (runs on its own thread) ────────────────────────────────
// Owns exclusive access to its gpiod_line; no locking needed.

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
  const TB6612Pins & pins, int pwm_freq, const std::string & chip_name,
  double min_pwm_fraction, double velocity_deadband)
: pins_(pins),
  pwm_freq_(pwm_freq > 0 ? pwm_freq : 1000),
  chip_name_(chip_name),
  min_pwm_fraction_(std::clamp(min_pwm_fraction, 0.0, 1.0)),
  velocity_deadband_(std::clamp(velocity_deadband, 0.0, 1.0))
{}

TB6612FNG::~TB6612FNG()
{
  shutdown();
}

// ── init ────────────────────────────────────────────────────────────────────

bool TB6612FNG::init()
{
  // Open the GPIO chip
  const std::string path = to_chip_path(chip_name_);
  chip_ = gpiod_chip_open(path.c_str());
  if (!chip_) {
    std::fprintf(stderr, "[TB6612FNG] Failed to open GPIO chip '%s'\n", path.c_str());
    return false;
  }

  // Helper lambda: get and request a single line as output (default LOW).
  auto req = [&](int offset, const char * name) -> gpiod_line * {
    gpiod_line * l = gpiod_chip_get_line(chip_, static_cast<unsigned int>(offset));
    if (!l) {
      std::fprintf(stderr, "[TB6612FNG] Failed to get line %d (%s)\n", offset, name);
      return nullptr;
    }
    if (gpiod_line_request_output(l, "hermes_driver", 0) < 0) {
      std::fprintf(stderr,
        "[TB6612FNG] Failed to request line %d (%s) as output\n", offset, name);
      return nullptr;
    }
    return l;
  };

  // ── Direction + standby pins ─────────────────────────────────────────────
  line_in1_a_ = req(pins_.motor_a.in1, "AIN1");
  if (!line_in1_a_) { gpiod_chip_close(chip_); chip_ = nullptr; return false; }

  line_in2_a_ = req(pins_.motor_a.in2, "AIN2");
  if (!line_in2_a_) {
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr; return false;
  }

  line_in1_b_ = req(pins_.motor_b.in1, "BIN1");
  if (!line_in1_b_) {
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr; return false;
  }

  line_in2_b_ = req(pins_.motor_b.in2, "BIN2");
  if (!line_in2_b_) {
    gpiod_line_release(line_in1_b_); line_in1_b_ = nullptr;
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr; return false;
  }

  line_stby_ = req(pins_.stby, "STBY");
  if (!line_stby_) {
    gpiod_line_release(line_in2_b_); line_in2_b_ = nullptr;
    gpiod_line_release(line_in1_b_); line_in1_b_ = nullptr;
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr; return false;
  }

  // ── PWM pins (each owned exclusively by its software-PWM thread) ─────────
  pwm_a_.line = request_output_line(chip_,
    static_cast<unsigned int>(pins_.motor_a.pwm), "hermes_driver_pwma");
  if (!pwm_a_.line) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMA line %d\n", pins_.motor_a.pwm);
    gpiod_line_release(line_stby_);   line_stby_  = nullptr;
    gpiod_line_release(line_in2_b_); line_in2_b_ = nullptr;
    gpiod_line_release(line_in1_b_); line_in1_b_ = nullptr;
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  pwm_b_.line = request_output_line(chip_,
    static_cast<unsigned int>(pins_.motor_b.pwm), "hermes_driver_pwmb");
  if (!pwm_b_.line) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMB line %d\n", pins_.motor_b.pwm);
    gpiod_line_release(pwm_a_.line);  pwm_a_.line = nullptr;
    gpiod_line_release(line_stby_);   line_stby_  = nullptr;
    gpiod_line_release(line_in2_b_); line_in2_b_ = nullptr;
    gpiod_line_release(line_in1_b_); line_in1_b_ = nullptr;
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  // ── Start software-PWM threads ────────────────────────────────────────────
  pwm_a_.freq    = pwm_freq_;
  pwm_a_.duty    = 0;
  pwm_a_.running = true;
  pwm_a_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_a_.duty), std::ref(pwm_a_.running),
    pwm_a_.line, pwm_freq_);

  pwm_b_.freq    = pwm_freq_;
  pwm_b_.duty    = 0;
  pwm_b_.running = true;
  pwm_b_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_b_.duty), std::ref(pwm_b_.running),
    pwm_b_.line, pwm_freq_);

  // ── Enable the driver (STBY HIGH) ─────────────────────────────────────────
  if (gpiod_line_set_value(line_stby_, 1) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to assert STBY pin %d\n", pins_.stby);
    pwm_a_.running = false;
    pwm_b_.running = false;
    if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
    if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }
    gpiod_line_release(pwm_b_.line);  pwm_b_.line = nullptr;
    gpiod_line_release(pwm_a_.line);  pwm_a_.line = nullptr;
    gpiod_line_release(line_stby_);   line_stby_  = nullptr;
    gpiod_line_release(line_in2_b_); line_in2_b_ = nullptr;
    gpiod_line_release(line_in1_b_); line_in1_b_ = nullptr;
    gpiod_line_release(line_in2_a_); line_in2_a_ = nullptr;
    gpiod_line_release(line_in1_a_); line_in1_a_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  initialised_ = true;
  return true;
}

// ── set_motor (private) ─────────────────────────────────────────────────────

void TB6612FNG::set_motor(
  gpiod_line * in1_line, gpiod_line * in2_line, PwmState & pwm, double speed)
{
  if (!initialised_) {return;}

  speed = std::clamp(speed, -1.0, 1.0);

  // Apply velocity deadband: commands smaller than this threshold are treated
  // as zero so that navigation micro-corrections do not snap the motor to the
  // minimum-PWM floor and cause oscillation / overshoot.
  if (std::abs(speed) < velocity_deadband_) {
    speed = 0.0;
  }

  // Direction logic:
  //   speed > 0  →  IN1 = HIGH, IN2 = LOW   (forward)
  //   speed < 0  →  IN1 = LOW,  IN2 = HIGH  (reverse)
  //   speed == 0 →  IN1 = LOW,  IN2 = LOW   (coast)
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

  if (gpiod_line_set_value(in1_line, in1_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN1\n");
  }
  if (gpiod_line_set_value(in2_line, in2_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN2\n");
  }

  // Compute PWM duty cycle (0–255).
  // For non-zero commands, remap the normalised speed from [0, 1] to
  // [min_pwm_fraction_, 1] so the motor always receives enough drive current
  // to overcome static friction while preserving proportional control.
  // Zero commands produce duty = 0 so the motor actually stops.
  const double abs_speed = std::abs(speed);
  unsigned duty;
  if (abs_speed < 1e-9) {
    duty = 0;
  } else {
    const double effective = min_pwm_fraction_ + (1.0 - min_pwm_fraction_) * abs_speed;
    duty = std::min(255u, static_cast<unsigned>(effective * 255.0));
  }
  pwm.duty = duty;
}

// ── Public motor setters ────────────────────────────────────────────────────

void TB6612FNG::set_motor_a(double speed)
{
  set_motor(line_in1_a_, line_in2_a_, pwm_a_, speed);
}

void TB6612FNG::set_motor_b(double speed)
{
  set_motor(line_in1_b_, line_in2_b_, pwm_b_, speed);
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
  // Stop software-PWM threads (safe even if never started)
  pwm_a_.running = false;
  pwm_b_.running = false;
  if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
  if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }

  if (!initialised_) {return;}

  // Drive direction pins and STBY LOW before releasing
  if (line_in1_a_) { gpiod_line_set_value(line_in1_a_, 0); }
  if (line_in2_a_) { gpiod_line_set_value(line_in2_a_, 0); }
  if (line_in1_b_) { gpiod_line_set_value(line_in1_b_, 0); }
  if (line_in2_b_) { gpiod_line_set_value(line_in2_b_, 0); }
  if (line_stby_)  { gpiod_line_set_value(line_stby_,  0); }

  // Release all line requests, then close the chip
  if (pwm_b_.line)  { gpiod_line_release(pwm_b_.line);  pwm_b_.line  = nullptr; }
  if (pwm_a_.line)  { gpiod_line_release(pwm_a_.line);  pwm_a_.line  = nullptr; }
  if (line_stby_)   { gpiod_line_release(line_stby_);   line_stby_   = nullptr; }
  if (line_in2_b_)  { gpiod_line_release(line_in2_b_);  line_in2_b_  = nullptr; }
  if (line_in1_b_)  { gpiod_line_release(line_in1_b_);  line_in1_b_  = nullptr; }
  if (line_in2_a_)  { gpiod_line_release(line_in2_a_);  line_in2_a_  = nullptr; }
  if (line_in1_a_)  { gpiod_line_release(line_in1_a_);  line_in1_a_  = nullptr; }
  gpiod_chip_close(chip_);
  chip_ = nullptr;
  initialised_ = false;
}

}  // namespace hermes_driver

