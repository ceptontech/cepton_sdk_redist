/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C++ interface for rapid prototyping.
*/
#pragma once

#include <algorithm>
#include <exception>
#include <thread>

#include "cepton_sdk.hpp"
#include "cepton_sdk_util.hpp"

namespace cepton_sdk {
namespace api {

/// Returns whether capture replay is not open.
inline bool is_live() { return !capture_replay::is_open(); }

/// Returns whether live or capture replay is running.
inline bool is_realtime() { return is_live() || capture_replay::is_running(); }

/// Returns whether capture replay is at the end and enable loop is false.
inline bool is_end() {
  if (capture_replay::is_open()) {
    if (capture_replay::get_enable_loop()) return false;
    return capture_replay::is_end();
  }
  return false;
}

/// Returns live or capture replay time.
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
    return CEPTON_PROCESS_ERROR(capture_replay::resume_blocking(t_length));
  }
}
}  // namespace internal

/// Sleeps or resumes capture replay for duration.
/**
 * If `t_length < 0`, then waits forever.
 */
inline SensorError wait(float t_length = -1.0f) {
  if (t_length >= 0.0f) {
    return CEPTON_PROCESS_ERROR(internal::wait(t_length));
  } else {
    do {
      CEPTON_RETURN_ERROR(internal::wait(0.1f));
    } while (!is_end());
    return CEPTON_SUCCESS;
  }
}

// -----------------------------------------------------------------------------
// Errors
// -----------------------------------------------------------------------------
/// DEPRECATED: use CEPTON_LOG_ERROR
inline SensorError log_error(const SensorError &error,
                             const std::string &msg = "") {
  return CEPTON_LOG_ERROR(error);
}

/// DEPRECATED: use CEPTON_CHECK_ERROR
inline SensorError check_error(const SensorError &error,
                               const std::string &msg = "") {
  return CEPTON_CHECK_ERROR(error);
}

/// Basic SDK error callback.
/**
 * Calls `CEPTON_LOG_ERROR`.
 */
inline void default_on_error(SensorHandle h, SensorErrorCode error_code,
                             const char *const error_msg,
                             const void *const error_data,
                             std::size_t error_data_size,
                             void *const instance) {
  CEPTON_LOG_ERROR(SensorError(error_code, error_msg));
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
/// Opens capture replay.
/**
 * If `enable_wait` is true, replays a few seconds to intialize sensors.
 */
inline SensorError open_replay(const std::string &capture_path,
                               bool enable_wait = false) {
  if (capture_replay::is_open()) CEPTON_RETURN_ERROR(capture_replay::close());
  CEPTON_RETURN_ERROR(capture_replay::open(capture_path));
  if (enable_wait) {
    CEPTON_RETURN_ERROR(capture_replay::resume_blocking(3.0f));
    CEPTON_RETURN_ERROR(capture_replay::seek(0.0f));
  }
  return CEPTON_SUCCESS;
}

/// Initialize SDK and optionally starts capture replay.
/**
 * If `enable_wait` is true, waits a few seconds to initialize sensors.
 */
inline SensorError initialize(Options options = create_options(),
                              const std::string &capture_path = "",
                              bool enable_wait = false) {
  // Initialize
  if (!capture_path.empty())
    options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  CEPTON_RETURN_ERROR(
      ::cepton_sdk::initialize(CEPTON_SDK_VERSION, options, default_on_error));

  if (capture_path.empty()) {
    if (enable_wait) wait(3.0f);
  } else {
    CEPTON_RETURN_ERROR(open_replay(capture_path, enable_wait));
  }

  return CEPTON_SUCCESS;
}

/// Returns whether indicated control flags are set.
inline bool has_control_flags(Control mask) {
  return (get_control_flags() & mask) == mask;
}

/// Enables/disables indicated control flags.
inline SensorError enable_control_flags(Control mask, bool tf) {
  if (tf) {
    return CEPTON_PROCESS_ERROR(set_control_flags(mask, mask));
  } else {
    return CEPTON_PROCESS_ERROR(set_control_flags(mask, 0));
  }
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
        ->
        operator()(handle, SensorError(error_code, error_msg));
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
  /// SensorImageFrameCallback class destructor.
  ~SensorImageFrameCallback() { deinitialize(); }
  /// Initializes SensorImageFrameCallback object.
  SensorError initialize() {
    CEPTON_RETURN_ERROR(listen_image_frames(global_on_callback, this));
    m_is_initialized = true;
    return CEPTON_SUCCESS;
  }
  /// Deinitializes SensorImageFrameCallback object.
  SensorError deinitialize() {
    clear();
    if (!m_is_initialized) return CEPTON_SUCCESS;
    if (::cepton_sdk::is_initialized()) unlisten_image_frames().ignore();
    m_is_initialized = false;
    return CEPTON_SUCCESS;
  }

  /// Returns true if SensorImageFrameCallback is initialized.
  bool is_initialized() const { return m_is_initialized; }

 private:
  bool m_is_initialized = false;
};

/// Callback for network packets.
/**
 * Must call `initialize` before use.
 */
class NetworkPacketCallback
    : public util::Callback<SensorHandle, int64_t, uint8_t const *,
                            std::size_t> {
 public:
  /// NetworkPacketCallback class destructor.
  ~NetworkPacketCallback() { deinitialize(); }
  /// Initialize NetworkPacketCallback object.
  SensorError initialize() {
    CEPTON_RETURN_ERROR(listen_network_packets(global_on_callback, this));
    m_is_initialized = true;
    return CEPTON_SUCCESS;
  }
  /// Deinitialize NetworkPacketCallback object.
  SensorError deinitialize() {
    clear();
    if (!m_is_initialized) return CEPTON_SUCCESS;
    if (::cepton_sdk::is_initialized()) unlisten_network_packets().ignore();
    m_is_initialized = false;
    return CEPTON_SUCCESS;
  }

 private:
  bool m_is_initialized = false;
};

// -----------------------------------------------------------------------------
// Sensors
// -----------------------------------------------------------------------------
/// Returns whether SDK has sensor with serial number.
inline bool has_sensor_by_serial_number(uint64_t serial_number) {
  SensorHandle handle;
  auto error = get_sensor_handle_by_serial_number(serial_number, handle);
  if (error) return false;
  return true;
}

/// Returns sensor information by serial number.
/**
 * Returns error if sensor not found.
 */
inline SensorError get_sensor_information_by_serial_number(
    uint64_t serial_number, SensorInformation &info) {
  CeptonSensorHandle handle;
  CEPTON_RETURN_ERROR(
      get_sensor_handle_by_serial_number(serial_number, handle));
  return CEPTON_PROCESS_ERROR(get_sensor_information(handle, info));
}

/// Returns serial numbers for all sensors.
inline std::vector<uint64_t> get_sensor_serial_numbers() {
  const int n_sensors = (int)get_n_sensors();
  std::vector<uint64_t> serial_numbers;
  serial_numbers.reserve(n_sensors);
  for (int i = 0; i < n_sensors; ++i) {
    SensorInformation sensor_info;
    if (CEPTON_LOG_ERROR(get_sensor_information_by_index(i, sensor_info)))
      continue;
    serial_numbers.push_back(sensor_info.serial_number);
  }
  std::sort(serial_numbers.begin(), serial_numbers.end());
  return serial_numbers;
}

}  // namespace api
}  // namespace cepton_sdk
