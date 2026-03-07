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
static gpiod_line_request * request_output_line(
  gpiod_chip * chip, unsigned int offset, const char * consumer)
{
  gpiod_line_settings * settings = gpiod_line_settings_new();
  if (!settings) {return nullptr;}
  gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

  gpiod_line_config * line_cfg = gpiod_line_config_new();
  if (!line_cfg) {
    gpiod_line_settings_free(settings);
    return nullptr;
  }
  if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) < 0) {
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    return nullptr;
  }

  gpiod_request_config * req_cfg = gpiod_request_config_new();
  if (!req_cfg) {
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    return nullptr;
  }
  gpiod_request_config_set_consumer(req_cfg, consumer);

  gpiod_line_request * req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

  gpiod_request_config_free(req_cfg);
  gpiod_line_config_free(line_cfg);
  gpiod_line_settings_free(settings);

  return req;  // nullptr on failure
}

// ── Software-PWM loop (runs on its own thread) ────────────────────────────────
// Owns exclusive access to its gpiod_line_request; no locking needed.

static void pwm_loop(
  std::atomic<unsigned> & duty, std::atomic<bool> & running,
  gpiod_line_request * request, unsigned int offset, int freq)
{
  const int period_us = 1000000 / freq;
  while (running.load(std::memory_order_relaxed)) {
    unsigned d = duty.load(std::memory_order_relaxed);

    if (d == 0) {
      gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_INACTIVE);
      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    } else if (d >= 255) {
      gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_ACTIVE);
      std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    } else {
      int on_us  = (period_us * static_cast<int>(d)) / 255;
      int off_us = period_us - on_us;
      gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_ACTIVE);
      std::this_thread::sleep_for(std::chrono::microseconds(on_us));
      gpiod_line_request_set_value(request, offset, GPIOD_LINE_VALUE_INACTIVE);
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
  // Open the GPIO chip (libgpiod v2 uses a full device path)
  const std::string path = to_chip_path(chip_name_);
  chip_ = gpiod_chip_open(path.c_str());
  if (!chip_) {
    std::fprintf(stderr, "[TB6612FNG] Failed to open GPIO chip '%s'\n", path.c_str());
    return false;
  }

  // ── Direction + standby pins (claimed as a group; main-thread only) ──────
  {
    unsigned int dir_offsets[5] = {
      static_cast<unsigned int>(pins_.motor_a.in1),
      static_cast<unsigned int>(pins_.motor_a.in2),
      static_cast<unsigned int>(pins_.motor_b.in1),
      static_cast<unsigned int>(pins_.motor_b.in2),
      static_cast<unsigned int>(pins_.stby),
    };

    gpiod_line_settings * settings = gpiod_line_settings_new();
    if (!settings) {
      gpiod_chip_close(chip_); chip_ = nullptr;
      return false;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    gpiod_line_config * line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
      gpiod_line_settings_free(settings);
      gpiod_chip_close(chip_); chip_ = nullptr;
      return false;
    }
    if (gpiod_line_config_add_line_settings(line_cfg, dir_offsets, 5, settings) < 0) {
      gpiod_line_config_free(line_cfg);
      gpiod_line_settings_free(settings);
      gpiod_chip_close(chip_); chip_ = nullptr;
      return false;
    }

    gpiod_request_config * req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
      gpiod_line_config_free(line_cfg);
      gpiod_line_settings_free(settings);
      gpiod_chip_close(chip_); chip_ = nullptr;
      return false;
    }
    gpiod_request_config_set_consumer(req_cfg, "hermes_driver");

    dir_request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);

    gpiod_request_config_free(req_cfg);
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);

    if (!dir_request_) {
      std::fprintf(stderr, "[TB6612FNG] Failed to request direction/standby lines\n");
      gpiod_chip_close(chip_); chip_ = nullptr;
      return false;
    }
  }

  // ── PWM pins (each gets its own request; owned exclusively by its thread) ─
  pwm_a_.offset  = static_cast<unsigned int>(pins_.motor_a.pwm);
  pwm_a_.freq    = pwm_freq_;
  pwm_a_.request = request_output_line(chip_, pwm_a_.offset, "hermes_driver_pwma");
  if (!pwm_a_.request) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMA line %d\n", pins_.motor_a.pwm);
    gpiod_line_request_release(dir_request_); dir_request_ = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  pwm_b_.offset  = static_cast<unsigned int>(pins_.motor_b.pwm);
  pwm_b_.freq    = pwm_freq_;
  pwm_b_.request = request_output_line(chip_, pwm_b_.offset, "hermes_driver_pwmb");
  if (!pwm_b_.request) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMB line %d\n", pins_.motor_b.pwm);
    gpiod_line_request_release(pwm_a_.request); pwm_a_.request = nullptr;
    gpiod_line_request_release(dir_request_);   dir_request_  = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  // ── Start software-PWM threads ────────────────────────────────────────────
  pwm_a_.duty    = 0;
  pwm_a_.running = true;
  pwm_a_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_a_.duty), std::ref(pwm_a_.running),
    pwm_a_.request, pwm_a_.offset, pwm_freq_);

  pwm_b_.duty    = 0;
  pwm_b_.running = true;
  pwm_b_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_b_.duty), std::ref(pwm_b_.running),
    pwm_b_.request, pwm_b_.offset, pwm_freq_);

  // ── Enable the driver (STBY HIGH) ─────────────────────────────────────────
  if (gpiod_line_request_set_value(
      dir_request_,
      static_cast<unsigned int>(pins_.stby),
      GPIOD_LINE_VALUE_ACTIVE) < 0)
  {
    std::fprintf(stderr, "[TB6612FNG] Failed to assert STBY pin %d\n", pins_.stby);
    pwm_a_.running = false;
    pwm_b_.running = false;
    if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
    if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }
    gpiod_line_request_release(pwm_b_.request); pwm_b_.request = nullptr;
    gpiod_line_request_release(pwm_a_.request); pwm_a_.request = nullptr;
    gpiod_line_request_release(dir_request_);   dir_request_  = nullptr;
    gpiod_chip_close(chip_); chip_ = nullptr;
    return false;
  }

  initialised_ = true;
  return true;
}

// ── set_motor (private) ─────────────────────────────────────────────────────

void TB6612FNG::set_motor(
  unsigned int in1_off, unsigned int in2_off, PwmState & pwm, double speed)
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
  gpiod_line_value in1_val, in2_val;
  if (speed > 0.0) {
    in1_val = GPIOD_LINE_VALUE_ACTIVE;
    in2_val = GPIOD_LINE_VALUE_INACTIVE;
  } else if (speed < 0.0) {
    in1_val = GPIOD_LINE_VALUE_INACTIVE;
    in2_val = GPIOD_LINE_VALUE_ACTIVE;
  } else {
    in1_val = GPIOD_LINE_VALUE_INACTIVE;
    in2_val = GPIOD_LINE_VALUE_INACTIVE;
  }

  if (gpiod_line_request_set_value(dir_request_, in1_off, in1_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN1 (offset %u)\n", in1_off);
  }
  if (gpiod_line_request_set_value(dir_request_, in2_off, in2_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN2 (offset %u)\n", in2_off);
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
  set_motor(
    static_cast<unsigned int>(pins_.motor_a.in1),
    static_cast<unsigned int>(pins_.motor_a.in2),
    pwm_a_, speed);
}

void TB6612FNG::set_motor_b(double speed)
{
  set_motor(
    static_cast<unsigned int>(pins_.motor_b.in1),
    static_cast<unsigned int>(pins_.motor_b.in2),
    pwm_b_, speed);
}

void TB6612FNG::set_motors(double speed_a, double speed_b)
{
  set_motor_a(speed_a);
  set_motor_b(speed_b);
}

void TB6612FNG::set_standby(bool enable)
{
  if (!initialised_) {return;}
  const gpiod_line_value val =
    enable ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
  if (gpiod_line_request_set_value(
      dir_request_, static_cast<unsigned int>(pins_.stby), val) < 0)
  {
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
  const gpiod_line_value off = GPIOD_LINE_VALUE_INACTIVE;
  gpiod_line_request_set_value(
    dir_request_, static_cast<unsigned int>(pins_.motor_a.in1), off);
  gpiod_line_request_set_value(
    dir_request_, static_cast<unsigned int>(pins_.motor_a.in2), off);
  gpiod_line_request_set_value(
    dir_request_, static_cast<unsigned int>(pins_.motor_b.in1), off);
  gpiod_line_request_set_value(
    dir_request_, static_cast<unsigned int>(pins_.motor_b.in2), off);
  gpiod_line_request_set_value(
    dir_request_, static_cast<unsigned int>(pins_.stby), off);

  // Release all requests, then close the chip
  if (pwm_b_.request) { gpiod_line_request_release(pwm_b_.request); pwm_b_.request = nullptr; }
  if (pwm_a_.request) { gpiod_line_request_release(pwm_a_.request); pwm_a_.request = nullptr; }
  if (dir_request_)   { gpiod_line_request_release(dir_request_);   dir_request_  = nullptr; }
  gpiod_chip_close(chip_);
  chip_ = nullptr;
  initialised_ = false;
}

}  // namespace hermes_driver

