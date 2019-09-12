/*
  Copyright Cepton Technologies Inc. 2017, All rights reserved.

  Cepton Sensor SDK C++ interface.
*/
#pragma once

#include "cepton_sdk.h"

#include <cassert>

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace cepton_sdk {

/**
 * - If `CEPTON_ENABLE_EXCEPTIONS` defined, terminates
 * - Otherwise, prints error message
 *
 * This function is static, just in case CEPTON_ENABLE_EXCEPTIONS is defined
 * differently in different translation units.
 */
static void throw_assert(const std::string &file, int line,
                         const std::string &condition, const std::string &msg) {
  if (msg.empty()) {
    std::fprintf(stderr, "AssertionError (file \"%s\", line %i, \"%s\")\n",
                 file.c_str(), line, condition.c_str());
  } else {
    std::fprintf(stderr,
                 "AssertionError (file \"%s\", line %i, \"%s\")\n\t%s\n",
                 file.c_str(), line, condition.c_str(), msg.c_str());
  }
#ifdef CEPTON_ENABLE_EXCEPTIONS
  std::terminate();
#endif
}

/// Runtime assert check for catching bugs.
/**
 * If condition is false, calls `throw_assert`.
 */
#define CEPTON_ASSERT(condition, msg)                                      \
  ((condition)                                                             \
       ? true                                                              \
       : (::cepton_sdk::throw_assert(__FILE__, __LINE__, #condition, msg), \
          false))

inline std::string get_version_string() {
  return cepton_sdk_get_version_string();
}
inline int get_version_major() { return cepton_sdk_get_version_major(); }
inline int get_version_minor() { return cepton_sdk_get_version_minor(); }

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
typedef CeptonSensorErrorCode SensorErrorCode;

inline std::string get_error_code_name(SensorErrorCode error_code) {
  return cepton_get_error_code_name(error_code);
}

inline bool is_error_code(SensorErrorCode error_code) {
  return (bool)cepton_is_error_code(error_code);
}

inline bool is_fault_code(SensorErrorCode error_code) {
  return (bool)cepton_is_fault_code(error_code);
}

/// Type checking for error callback data. Not implemented.
/**
 * If specified type is correct, returns pointer to data, otherwise returns
 * nullptr.
 */
template <typename T>
const T *get_error_data(SensorErrorCode error_code, const void *error_data,
                        std::size_t error_data_size) {
  if (error_data_size == 0) {
    return nullptr;
  }

  switch (error_code) {
    default:
      return nullptr;
  }

  return dynamic_cast<const T *>(error_data);
}

/// Error returned by most functions.
/**
 * Implicitly convertible from/to SensorErrorCode.
 * Getter functions do not return an error, because they cannot fail.
 * Will call `CEPTON_RUNTIME_ASSERT` if nonzero error is not used (call
 * `ignore` to manually use error).
 */
class SensorError : public std::runtime_error {
 public:
  SensorError(SensorErrorCode code_, const std::string &msg_)
      : std::runtime_error(create_message(code_, msg_)),
        m_code(code_),
        m_msg(msg_) {
    CEPTON_ASSERT(!get_error_code_name(code_).empty(),
                  std::string("Invalid error code: ") + std::to_string(m_code));
  }
  SensorError(SensorErrorCode code_) : SensorError(code_, "") {}
  SensorError() : SensorError(CEPTON_SUCCESS) {}
  ~SensorError() { check_used(); }

  SensorError(const SensorError &other) : std::runtime_error(other) {
    m_code = other.code();
    m_msg = other.msg();
  }
  SensorError &operator=(const SensorError &other) {
    check_used();
    std::runtime_error::operator=(other);
    m_code = other.code();
    m_msg = other.msg();
    m_used = false;
    return *this;
  }

  /// Internal use only.
  bool used() const { return m_used; }

  /// Mark error as used.
  const SensorError &ignore() const {
    m_used = true;
    return *this;
  }

  /// Returns error message;
  const std::string &msg() const {
    m_used = true;
    return m_msg;
  }

  /// Returns error code
  SensorErrorCode code() const {
    m_used = true;
    return m_code;
  }

  /// Implicitly convert to `SensorErrorCode`.
  operator SensorErrorCode() const { return code(); }

  /// Returns `false` if code is `CEPTON_SUCCESS`, true otherwise.
  operator bool() const { return code(); }

  const std::string name() const { return get_error_code_name(code()); }

  bool is_error() const { return is_error_code(code()); }
  bool is_fault() const { return is_fault_code(code()); }

 private:
  static std::string create_message(SensorErrorCode code,
                                    const std::string &msg) {
    if (!code) return "";
    std::string result = get_error_code_name(code);
    if (!msg.empty()) {
      result.append("\n\t");
      result.append(msg);
    }
    return result;
  }

  /// Throw assert if error not used.
  void check_used() const {
    CEPTON_ASSERT(!m_code || m_used, std::string("Error not used: ") + what());
  }

 private:
  SensorErrorCode m_code;
  std::string m_msg;
  mutable bool m_used = false;
};

/// Wrapper for adding current context to error stack traces.
class SensorErrorWrapper {
 public:
  SensorErrorWrapper(const std::string &context_) : m_context(context_) {}

  SensorErrorWrapper &operator=(const SensorError &error_) {
    if (enable_accumulation && m_error) return *this;
    if (!error_) {
      m_error = SensorError();
      return *this;
    }

    // Prepend context.
    std::string msg = m_context;
    if (!error_.msg().empty()) {
      msg.append("\n\t");
      msg.append(error_.msg());
    }

    m_error = SensorError(error_.code(), msg);
    return *this;
  }

  operator bool() const { return m_error; }
  const SensorError &error() const { return m_error; }
  operator const SensorError &() const { return m_error; }

 public:
  /// If true, store first stored error, and ignore future errors.
  bool enable_accumulation = true;

 private:
  const std::string m_context;
  SensorError m_error;
};

/// Returns and clears the last sdk error.
/**
 * Called automatically by all C++ methods, so only useful when calling C
 * methods directly.
 */
inline SensorError get_error() {
  const char *error_msg;
  const auto error_code = cepton_sdk_get_error(&error_msg);
  return SensorError(error_code, error_msg);
}

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef CeptonSensorHandle SensorHandle;
const SensorHandle SENSOR_HANDLE_FLAG_MOCK = CEPTON_SENSOR_HANDLE_FLAG_MOCK;

typedef CeptonSensorModel SensorModel;

typedef CeptonSensorInformation SensorInformation;

typedef CeptonSensorImagePoint SensorImagePoint;

//------------------------------------------------------------------------------
// SDK Setup
//------------------------------------------------------------------------------
typedef CeptonSDKControl Control;
typedef CeptonSDKFrameMode FrameMode;
typedef CeptonSDKFrameOptions FrameOptions;
typedef CeptonSDKOptions Options;

inline FrameOptions create_frame_options() {
  return cepton_sdk_create_frame_options();
}

inline Options create_options() { return cepton_sdk_create_options(); }

typedef void (*FpSensorErrorCallback)(SensorHandle handle,
                                      SensorErrorCode error_code,
                                      const char *error_msg,
                                      const void *error_data,
                                      size_t error_data_size, void *user_data);

inline bool is_initialized() { return (bool)cepton_sdk_is_initialized(); }
inline SensorError initialize(int version,
                              const Options &options = create_options(),
                              const FpSensorErrorCallback &cb = nullptr,
                              void *const user_data = nullptr) {
  cepton_sdk_initialize(version, &options, cb, user_data);
  return get_error();
}
inline SensorError deinitialize() {
  cepton_sdk_deinitialize();
  return get_error();
}

inline SensorError clear() {
  cepton_sdk_clear();
  return get_error();
}

inline SensorError set_control_flags(Control mask, Control flags) {
  cepton_sdk_set_control_flags(mask, flags);
  return get_error();
}

inline Control get_control_flags() { return cepton_sdk_get_control_flags(); }

inline bool has_control_flag(Control flag) {
  return (bool)cepton_sdk_has_control_flag(flag);
}

inline SensorError set_port(uint16_t port) {
  cepton_sdk_set_port(port);
  return get_error();
}
inline uint16_t get_port() { return cepton_sdk_get_port(); }

inline SensorError set_frame_options(const CeptonSDKFrameOptions &options) {
  cepton_sdk_set_frame_options(&options);
  return get_error();
}
inline FrameMode get_frame_mode() { return cepton_sdk_get_frame_mode(); }
inline float get_frame_length() { return cepton_sdk_get_frame_length(); }

//------------------------------------------------------------------------------
// Points
//------------------------------------------------------------------------------
typedef void (*FpSensorImageDataCallback)(SensorHandle handle,
                                          std::size_t n_points,
                                          const SensorImagePoint *c_points,
                                          void *user_data);

inline SensorError listen_image_frames(FpSensorImageDataCallback cb,
                                       void *const user_data) {
  cepton_sdk_listen_image_frames(cb, user_data);
  return get_error();
}
inline SensorError unlisten_image_frames() {
  cepton_sdk_unlisten_image_frames();
  return get_error();
}

//------------------------------------------------------------------------------
// Sensors
//------------------------------------------------------------------------------
inline std::size_t get_n_sensors() { return cepton_sdk_get_n_sensors(); }

inline SensorError get_sensor_handle_by_serial_number(uint64_t serial_number,
                                                      SensorHandle &handle) {
  cepton_sdk_get_sensor_handle_by_serial_number(serial_number, &handle);
  return get_error();
}

inline SensorError get_sensor_information_by_index(std::size_t idx,
                                                   SensorInformation &info) {
  cepton_sdk_get_sensor_information_by_index(idx, &info);
  return get_error();
}
inline SensorError get_sensor_information(SensorHandle handle,
                                          SensorInformation &info) {
  cepton_sdk_get_sensor_information(handle, &info);
  return get_error();
}

//------------------------------------------------------------------------------
// Serial
//------------------------------------------------------------------------------
typedef void (*FpSerialReceiveCallback)(SensorHandle handle, const char *str,
                                        void *user_data);

inline SensorError listen_serial_lines(FpSerialReceiveCallback cb,
                                       void *const user_data) {
  cepton_sdk_listen_serial_lines(cb, user_data);
  return get_error();
}
inline SensorError unlisten_serial_lines() {
  cepton_sdk_unlisten_serial_lines();
  return get_error();
}

//------------------------------------------------------------------------------
// Networking
//------------------------------------------------------------------------------
typedef void (*FpNetworkReceiveCallback)(SensorHandle handle, int64_t timestamp,
                                         const uint8_t *buffer,
                                         size_t buffer_size, void *user_data);

inline SensorError listen_network_packets(FpNetworkReceiveCallback cb,
                                          void *const user_data) {
  cepton_sdk_listen_network_packet(cb, user_data);
  return get_error();
}
inline SensorError unlisten_network_packets() {
  cepton_sdk_unlisten_network_packet();
  return get_error();
}

inline SensorError mock_network_receive(SensorHandle handle, int64_t timestamp,
                                        const uint8_t *const buffer,
                                        std::size_t buffer_size) {
  cepton_sdk_mock_network_receive(handle, timestamp, buffer, buffer_size);
  return get_error();
}
//------------------------------------------------------------------------------
// Capture Replay
//------------------------------------------------------------------------------
namespace capture_replay {

inline bool is_open() { return (bool)cepton_sdk_capture_replay_is_open(); }
inline SensorError open(const std::string &path) {
  cepton_sdk_capture_replay_open(path.c_str());
  return get_error();
}
inline SensorError close() {
  cepton_sdk_capture_replay_close();
  return get_error();
}

inline std::string get_filename() {
  return cepton_sdk_capture_replay_get_filename();
}

inline uint64_t get_start_time() {
  return cepton_sdk_capture_replay_get_start_time();
}
inline float get_position() { return cepton_sdk_capture_replay_get_position(); }
/// Returns capture file time (unix time [microseconds])
inline uint64_t get_time() {
  return get_start_time() + uint64_t(1e6 * get_position());
}
inline float get_length() { return cepton_sdk_capture_replay_get_length(); }

inline bool is_end() { return (bool)cepton_sdk_capture_replay_is_end(); }

inline SensorError seek(float position) {
  cepton_sdk_capture_replay_seek(position);
  return get_error();
}
/// Seek to relative capture file position [seconds].
/**
 * Returns error if position is invalid.
 */
inline SensorError seek_relative(float position) {
  position += get_position();
  cepton_sdk_capture_replay_seek(position);
  return get_error();
}

inline SensorError set_enable_loop(bool value) {
  cepton_sdk_capture_replay_set_enable_loop((int)value);
  return get_error();
}
inline bool get_enable_loop() {
  return (bool)cepton_sdk_capture_replay_get_enable_loop();
}

inline SensorError set_speed(float speed) {
  cepton_sdk_capture_replay_set_speed(speed);
  return get_error();
}
inline float get_speed() { return cepton_sdk_capture_replay_get_speed(); }

inline SensorError resume_blocking_once() {
  cepton_sdk_capture_replay_resume_blocking_once();
  return get_error();
}
inline SensorError resume_blocking(float duration) {
  cepton_sdk_capture_replay_resume_blocking(duration);
  return get_error();
}
inline bool is_running() { return cepton_sdk_capture_replay_is_running(); }
inline SensorError resume() {
  cepton_sdk_capture_replay_resume();
  return get_error();
}
inline SensorError pause() {
  cepton_sdk_capture_replay_pause();
  return get_error();
}

}  // namespace capture_replay
}  // namespace cepton_sdk
