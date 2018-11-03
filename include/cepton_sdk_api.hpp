#pragma once

#include <algorithm>
#include <exception>
#include <thread>

#include "cepton_sdk.hpp"
#include "cepton_sdk_util.hpp"

namespace cepton_sdk {
namespace api {

#include "cepton_sdk_def.h"

/// Returns true if capture replay is not open.
inline bool is_live() { return !capture_replay::is_open(); }

/// Returns true if live or capture replay is running.
inline bool is_realtime() { return is_live() || capture_replay::is_running(); }

inline bool is_end() {
  if (capture_replay::is_open()) {
    if (capture_replay::get_enable_loop()) return false;
    return capture_replay::is_end();
  }
  return false;
}

/// Returns capture replay time or live time.
inline int64_t get_time() {
  return (is_live()) ? util::get_timestamp_usec() : capture_replay::get_time();
}

namespace internal {
inline SensorError wait(float t_length) {
  if (is_realtime()) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds((int)(1e3f * t_length)));
    return CEPTON_SUCCESS;
  } else {
    return capture_replay::resume_blocking(t_length);
  }
}
}  // namespace internal

/// Sleeps or resumes capture replay for duration.
/**
 * If `t_length < 0`, then waits forever.
 */
inline SensorError wait(float t_length = -1.0f) {
  if (t_length >= 0.0f) {
    return internal::wait(t_length);
  } else {
    do {
      const auto error = internal::wait(0.1f);
      if (error) return error;
    } while (!is_end());
  }
  return CEPTON_SUCCESS;
}

// -----------------------------------------------------------------------------
// Errors
// -----------------------------------------------------------------------------
namespace internal {
inline SensorError create_error(SensorErrorCode error_code,

                                const std::string &msg = "") {
  if (!error_code) return SensorError();

  std::string error_code_name = get_error_code_name(error_code);
  char error_msg[1024];
  if (msg.empty()) {
    std::snprintf(error_msg, 1024, "SDK Error: %s!\n", error_code_name.c_str());
  } else {
    std::snprintf(error_msg, 1024, "%s: %s!\n", msg.c_str(),
                  error_code_name.c_str());
  }
  return SensorError(error_code, error_msg);
}
}  // namespace internal

/// DEPRECATED: use `cepton_sdk::api::log_error`.
DEPRECATED inline SensorError log_error_code(SensorErrorCode error_code,
                                             const std::string &msg = "") {
  const auto error = internal::create_error(error_code, msg);
  if (!error) return error;

  std::fprintf(stderr, "%s\n", error.what());
  return error;
}

/// DEPRECATED: use `cepton_sdk::api::check_error`.
DEPRECATED inline SensorError check_error_code(SensorErrorCode error_code,
                                               const std::string &msg = "") {
  const auto error = internal::create_error(error_code, msg);
  if (!error) return error;

  if (error.is_error()) {
    throw error;
  } else {
    std::fprintf(stderr, "%s\n", error.what());
  }
  return error;
}

/// Prints error.
inline const SensorError &log_error(const SensorError &error,
                                    const std::string &msg = "") {
  if (!error) return error;

  if (msg.empty()) {
    std::fprintf(stderr, "%s\n", error.what());
  } else {
    std::fprintf(stderr, "%s <%s>\n", msg.c_str(), error.what());
  }
  return error;
}

/// Handles error.
/**
 * If error, raises exception.
 * Otherwise, prints error.
 */
inline const SensorError &check_error(const SensorError &error,
                                      const std::string &msg = "") {
  if (!error) return error;

  if (error.is_error()) {
    throw error;
  } else {
    log_error(error, msg);
  }
  return error;
}

/// Basic SDK error callback.
/**
 * Calls `cepton_sdk::api::check_error_code`.
 */
inline void default_on_error(SensorHandle h, SensorErrorCode error_code,
                             const char *const error_msg,
                             const void *const error_data,
                             std::size_t error_data_size,
                             void *const instance) {
  log_error(SensorError(error_code, error_msg));
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
/// Opens capture replay.
inline SensorError open_replay(const std::string &capture_path) {
  SensorError error;
  if (capture_replay::is_open()) {
    error = capture_replay::close();
    if (error) return error;
  }
  error = capture_replay::open(capture_path);
  if (error) return error;
  error = capture_replay::resume_blocking(10.0f);
  if (error) return error;
  error = capture_replay::seek(0.0f);
  if (error) return error;
  return CEPTON_SUCCESS;
}

/// Initialize SDK and optionally starts capture replay.
inline SensorError initialize(Options options = create_options(),
                              const std::string &capture_path = "") {
  // Initialize
  if (!capture_path.empty())
    options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  auto error =
      ::cepton_sdk::initialize(CEPTON_SDK_VERSION, options, default_on_error);
  if (error) return error;

  // Open capture replay
  if (!capture_path.empty()) {
    error = open_replay(capture_path);
    if (error) return error;
  }
  return CEPTON_SUCCESS;
}

inline bool has_control_flags(Control flags) {
  return (get_control_flags() & flags) == flags;
}

inline void enable_control_flags(Control flags) {
  set_control_flags(flags, flags);
}

inline void disable_control_flags(Control flags) {
  set_control_flags(flags, 0);
}

/// Callback for sensor errors.
class SensorErrorCallback
    : public util::Callback<SensorHandle, const SensorError &> {
 public:
  static void global_on_callback(SensorHandle handle,
                                 SensorErrorCode error_code,
                                 const char *error_msg,
                                 const void *const error_data,
                                 size_t error_data_size, void *const instance) {
    ((const SensorErrorCallback *)instance)
        ->emit(handle, SensorError(error_code, error_msg));
  }
};

/// Callback for image frames.
/**
 * Must call `initialize` before use.
 */
class SensorImageFrameCallback

    : public util::Callback<SensorHandle, std::size_t,
                            const SensorImagePoint *> {
 public:
  ~SensorImageFrameCallback() { deinitialize(); }
  SensorError initialize() {
    return listen_image_frames(global_on_callback, this);
  }
  SensorError deinitialize() {
    if (!is_initialized()) return CEPTON_SUCCESS;
    return unlisten_image_frames();
  }
};

/// Callback for network packets.
/**
 * Must call `initialize` before use.
 */
class NetworkPacketCallback
    : public util::Callback<SensorHandle, int64_t, uint8_t const *,
                            std::size_t> {
 public:
  ~NetworkPacketCallback() { deinitialize(); }
  SensorError initialize() {
    return listen_network_packets(global_on_callback, this);
  }
  SensorError deinitialize() {
    if (!is_initialized()) return CEPTON_SUCCESS;
    return unlisten_network_packets();
  }
};

// -----------------------------------------------------------------------------
// Sensors
// -----------------------------------------------------------------------------
inline bool has_sensor_by_serial_number(uint64_t serial_number) {
  SensorHandle handle;
  auto error = get_sensor_handle_by_serial_number(serial_number, handle);
  if (error) return false;
  return true;
}

/**
 * Returns error if sensor not found.
 */
inline SensorError get_sensor_information_by_serial_number(
    uint64_t serial_number, SensorInformation &info) {
  CeptonSensorHandle handle;
  auto error = get_sensor_handle_by_serial_number(serial_number, handle);
  if (error) return error;
  return get_sensor_information(handle, info);
}

/// Returns serial numbers for all sensors.
inline std::vector<uint64_t> get_sensor_serial_numbers() {
  const int n_sensors = get_n_sensors();
  std::vector<uint64_t> serial_numbers;
  serial_numbers.reserve(n_sensors);
  for (int i = 0; i < n_sensors; ++i) {
    SensorInformation sensor_info;
    auto error = get_sensor_information_by_index(i, sensor_info);
    log_error(error);
    if (error) continue;
    serial_numbers.push_back(sensor_info.serial_number);
  }
  std::sort(serial_numbers.begin(), serial_numbers.end());
  return serial_numbers;
}

#include "cepton_sdk_undef.h"

}  // namespace api
}  // namespace cepton_sdk
