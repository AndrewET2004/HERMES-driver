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

/// Open @p chip and request a single output line at @p offset as consumer
/// @p consumer.  Returns the new gpiod_line_request on success, nullptr on
/// failure.  All temporary configuration objects are freed before returning.
static gpiod_line_request * request_output_line(
  gpiod_chip * chip, unsigned int offset, const char * consumer)
{
  gpiod_line_request * result = nullptr;

  gpiod_line_settings * settings = gpiod_line_settings_new();
  if (!settings) {return nullptr;}

  gpiod_line_config * line_cfg = gpiod_line_config_new();
  if (!line_cfg) {
    gpiod_line_settings_free(settings);
    return nullptr;
  }

  gpiod_request_config * req_cfg = gpiod_request_config_new();
  if (!req_cfg) {
    gpiod_line_config_free(line_cfg);
    gpiod_line_settings_free(settings);
    return nullptr;
  }

  gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

  if (gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings) == 0) {
    gpiod_request_config_set_consumer(req_cfg, consumer);
    result = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
  }

  gpiod_request_config_free(req_cfg);
  gpiod_line_config_free(line_cfg);
  gpiod_line_settings_free(settings);
  return result;
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
  // Open the GPIO chip
  const std::string path = to_chip_path(chip_name_);
  gpiod_chip * chip = gpiod_chip_open(path.c_str());
  if (!chip) {
    std::fprintf(stderr, "[TB6612FNG] Failed to open GPIO chip '%s'\n", path.c_str());
    return false;
  }

  // Helper lambda: request a single output line and fill a GpioLine struct.
  auto req = [&](int offset_i, const char * name, GpioLine & out) -> bool {
    unsigned int off = static_cast<unsigned int>(offset_i);
    out.request = request_output_line(chip, off, "hermes_driver");
    if (!out.request) {
      std::fprintf(stderr,
        "[TB6612FNG] Failed to request line %d (%s) as output\n", offset_i, name);
      return false;
    }
    out.offset = off;
    return true;
  };

  // ── Release helper: release a GpioLine if it has an active request ────────
  auto rel = [](GpioLine & gl) {
    if (gl.request) {
      gpiod_line_request_release(gl.request);
      gl.request = nullptr;
    }
  };

  // ── Direction + standby pins ─────────────────────────────────────────────
  if (!req(pins_.motor_a.in1, "AIN1", line_in1_a_)) {
    gpiod_chip_close(chip); return false;
  }

  if (!req(pins_.motor_a.in2, "AIN2", line_in2_a_)) {
    rel(line_in1_a_); gpiod_chip_close(chip); return false;
  }

  if (!req(pins_.motor_b.in1, "BIN1", line_in1_b_)) {
    rel(line_in2_a_); rel(line_in1_a_);
    gpiod_chip_close(chip); return false;
  }

  if (!req(pins_.motor_b.in2, "BIN2", line_in2_b_)) {
    rel(line_in1_b_); rel(line_in2_a_); rel(line_in1_a_);
    gpiod_chip_close(chip); return false;
  }

  if (!req(pins_.stby, "STBY", line_stby_)) {
    rel(line_in2_b_); rel(line_in1_b_); rel(line_in2_a_); rel(line_in1_a_);
    gpiod_chip_close(chip); return false;
  }

  // ── PWM pins (each owned exclusively by its software-PWM thread) ─────────
  pwm_a_.gpio.request = request_output_line(chip,
    static_cast<unsigned int>(pins_.motor_a.pwm), "hermes_driver_pwma");
  if (!pwm_a_.gpio.request) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMA line %d\n", pins_.motor_a.pwm);
    rel(line_stby_); rel(line_in2_b_); rel(line_in1_b_);
    rel(line_in2_a_); rel(line_in1_a_);
    gpiod_chip_close(chip);
    return false;
  }
  pwm_a_.gpio.offset = static_cast<unsigned int>(pins_.motor_a.pwm);

  pwm_b_.gpio.request = request_output_line(chip,
    static_cast<unsigned int>(pins_.motor_b.pwm), "hermes_driver_pwmb");
  if (!pwm_b_.gpio.request) {
    std::fprintf(stderr, "[TB6612FNG] Failed to request PWMB line %d\n", pins_.motor_b.pwm);
    rel(pwm_a_.gpio); rel(line_stby_); rel(line_in2_b_); rel(line_in1_b_);
    rel(line_in2_a_); rel(line_in1_a_);
    gpiod_chip_close(chip);
    return false;
  }
  pwm_b_.gpio.offset = static_cast<unsigned int>(pins_.motor_b.pwm);

  // The chip can be closed once all line requests have been obtained.
  gpiod_chip_close(chip);

  // ── Start software-PWM threads ────────────────────────────────────────────
  pwm_a_.freq    = pwm_freq_;
  pwm_a_.duty    = 0;
  pwm_a_.running = true;
  pwm_a_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_a_.duty), std::ref(pwm_a_.running),
    pwm_a_.gpio.request, pwm_a_.gpio.offset, pwm_freq_);

  pwm_b_.freq    = pwm_freq_;
  pwm_b_.duty    = 0;
  pwm_b_.running = true;
  pwm_b_.thread  = std::thread(
    pwm_loop,
    std::ref(pwm_b_.duty), std::ref(pwm_b_.running),
    pwm_b_.gpio.request, pwm_b_.gpio.offset, pwm_freq_);

  // ── Enable the driver (STBY HIGH) ─────────────────────────────────────────
  if (gpiod_line_request_set_value(line_stby_.request, line_stby_.offset,
    GPIOD_LINE_VALUE_ACTIVE) < 0)
  {
    std::fprintf(stderr, "[TB6612FNG] Failed to assert STBY pin %d\n", pins_.stby);
    pwm_a_.running = false;
    pwm_b_.running = false;
    if (pwm_a_.thread.joinable()) { pwm_a_.thread.join(); }
    if (pwm_b_.thread.joinable()) { pwm_b_.thread.join(); }
    gpiod_line_request_release(pwm_b_.gpio.request); pwm_b_.gpio.request = nullptr;
    gpiod_line_request_release(pwm_a_.gpio.request); pwm_a_.gpio.request = nullptr;
    rel(line_stby_); rel(line_in2_b_); rel(line_in1_b_);
    rel(line_in2_a_); rel(line_in1_a_);
    return false;
  }

  initialised_ = true;
  return true;
}

// ── set_motor (private) ─────────────────────────────────────────────────────

void TB6612FNG::set_motor(
  const GpioLine & in1, const GpioLine & in2, PwmState & pwm, double speed)
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

  if (gpiod_line_request_set_value(in1.request, in1.offset, in1_val) < 0) {
    std::fprintf(stderr, "[TB6612FNG] Failed to write IN1\n");
  }
  if (gpiod_line_request_set_value(in2.request, in2.offset, in2_val) < 0) {
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
  if (gpiod_line_request_set_value(line_stby_.request, line_stby_.offset,
    enable ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) < 0)
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
  auto set_low = [](const GpioLine & gl) {
    if (gl.request) {
      gpiod_line_request_set_value(gl.request, gl.offset, GPIOD_LINE_VALUE_INACTIVE);
    }
  };
  set_low(line_in1_a_);
  set_low(line_in2_a_);
  set_low(line_in1_b_);
  set_low(line_in2_b_);
  set_low(line_stby_);

  // Release all line requests
  auto rel = [](GpioLine & gl) {
    if (gl.request) {
      gpiod_line_request_release(gl.request);
      gl.request = nullptr;
    }
  };
  rel(pwm_b_.gpio);
  rel(pwm_a_.gpio);
  rel(line_stby_);
  rel(line_in2_b_);
  rel(line_in1_b_);
  rel(line_in2_a_);
  rel(line_in1_a_);

  initialised_ = false;
}

}  // namespace hermes_driver

