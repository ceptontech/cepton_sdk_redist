/*
  Copyright Cepton Technologies Inc., All rights reserved.

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
//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
namespace internal {
std::string create_context_message(const std::string &file, int line,
                                   const std::string &code);
std::string create_assert_message(const std::string &file, int line,
                                  const std::string &code,
                                  const std::string &msg);
void throw_assert(const std::string &file, int line, const std::string &code,
                  const std::string &msg);
}  // namespace internal

/// Runtime assert check for catching bugs.
/**
 * - If `CEPTON_ENABLE_EXCEPTIONS` defined, terminates.
 * - Otherwise, prints message.
 */
#define CEPTON_ASSERT(condition, msg)                                      \
  ((condition) ? true                                                      \
               : (::cepton_sdk::internal::throw_assert(__FILE__, __LINE__, \
                                                       #condition, msg),   \
                  false))

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

/// Type checking for error callback data.
/**
 * NOT IMPLEMENTED
 *
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
 * Implicitly convertible from/to `SensorErrorCode`.
 * Getter functions do not return an error, because they cannot fail.
 * Will call `CEPTON_ASSERT` if nonzero error is not used (call
 * `ignore` to manually use error).
 */
class SensorError : public std::runtime_error {
 public:
  /// Create SensorError class object with SensorErrorCode and error message.
  SensorError(SensorErrorCode code, const std::string &msg);

  /// Create SensorError class object with error code.
  SensorError(SensorErrorCode code);

  /// SensorError class default constructor
  SensorError();

  /// SensorError class destructor
  ~SensorError();

  /// Create SensorError object using SensorError object.
  SensorError(const SensorError &other);

  /// SensorError class assignment operator
  SensorError &operator=(const SensorError &other);

  /// Internal use only.
  bool used() const;

  /// Mark error as used.
  const SensorError &ignore() const;

  const char *what() const noexcept override;

  /// Returns error message.
  const std::string &msg() const;

  /// Returns error code.
  SensorErrorCode code() const;

  /// Implicitly convert to `SensorErrorCode`.
  operator SensorErrorCode() const;

  /// Returns `false` if error code is `CEPTON_SUCCESS`, true otherwise.
  operator bool() const;

  /// Returns error code name.
  const std::string name() const;

  /// Returns true if parent object is an error.
  bool is_error() const;

  /// Returns true if parent object is a fault.
  bool is_fault() const;

 private:
  static std::string create_message(SensorErrorCode code,
                                    const std::string &msg);
  void check_used() const;

 private:
  SensorErrorCode m_code;
  std::string m_msg;
  mutable bool m_used = false;
};

namespace internal {
SensorError add_error_context(const SensorError &error,
                              const std::string &context);
}  // namespace internal

/// Wrapper for adding current context to error stack traces.
class SensorErrorWrapper {
 public:
  SensorErrorWrapper(const std::string &context);
  SensorErrorWrapper &operator=(const SensorError &error);

  operator bool() const;
  const SensorError &error() const;
  operator const SensorError &() const;

 public:
  /// If true, store first stored error, and ignore future errors.
  bool enable_accumulation = true;

 private:
  const std::string m_context;
  SensorError m_error;
};

#define CEPTON_ASSERT_ERROR(condition, code, msg)            \
  if (!(condition))                                          \
    return ::cepton_sdk::SensorError(                        \
        code, ::cepton_sdk::internal::create_assert_message( \
                  __FILE__, __LINE__, #condition, msg));

namespace internal {
SensorError process_error(const std::string &file, int line,
                          const std::string &code, const SensorError &error,
                          bool enable_log, bool enable_raise);
}  // namespace internal

/// Add context to error.
#define CEPTON_PROCESS_ERROR(code)                                       \
  ::cepton_sdk::internal::process_error(__FILE__, __LINE__, #code, code, \
                                        false, false)

/// If error, raise.
#define CEPTON_CHECK_ERROR(code)                                         \
  ::cepton_sdk::internal::process_error(__FILE__, __LINE__, #code, code, \
                                        false, true)

/// If error, print.
#define CEPTON_LOG_ERROR(code)                                                 \
  ::cepton_sdk::internal::process_error(__FILE__, __LINE__, #code, code, true, \
                                        false)                                 \
      .ignore()

/// If error, return.
#define CEPTON_RETURN_ERROR(code)                                              \
  do {                                                                         \
    const auto cepton_return_error_error =                                     \
        ::cepton_sdk::internal::process_error(__FILE__, __LINE__, #code, code, \
                                              false, false);                   \
    if (cepton_return_error_error) return cepton_return_error_error;           \
  } while (0)

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

#define CEPTON_SDK_WRAP_C(code)                                           \
  (code, ::cepton_sdk::internal::process_error(__FILE__, __LINE__, #code, \
                                               get_error(), false, false))

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef CeptonSensorHandle SensorHandle;
const SensorHandle SENSOR_HANDLE_FLAG_MOCK = CEPTON_SENSOR_HANDLE_FLAG_MOCK;
typedef CeptonSensorModel SensorModel;

inline bool is_sora(SensorModel model) { return cepton_is_sora(model); }
inline bool is_hr80(SensorModel model) { return cepton_is_hr80(model); }
inline bool is_vista(SensorModel model) { return cepton_is_vista(model); }

typedef CeptonSensorInformation SensorInformation;
typedef CeptonSensorImagePoint SensorImagePoint;

//------------------------------------------------------------------------------
// SDK Setup
//------------------------------------------------------------------------------
inline std::string get_version_string() {
  return cepton_sdk_get_version_string();
}
inline int get_version_major() { return cepton_sdk_get_version_major(); }
inline int get_version_minor() { return cepton_sdk_get_version_minor(); }
inline int get_version_patch() { return cepton_sdk_get_version_patch(); }

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
  return CEPTON_SDK_WRAP_C(
      cepton_sdk_initialize(version, &options, cb, user_data));
}
inline SensorError deinitialize() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_deinitialize());
}

inline SensorError clear() { return CEPTON_SDK_WRAP_C(cepton_sdk_clear()); }

inline SensorError set_control_flags(Control mask, Control flags) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_set_control_flags(mask, flags));
}
inline Control get_control_flags() { return cepton_sdk_get_control_flags(); }
inline bool has_control_flag(Control flag) {
  return (bool)cepton_sdk_has_control_flag(flag);
}

inline SensorError set_port(uint16_t port) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_set_port(port));
}
inline uint16_t get_port() { return cepton_sdk_get_port(); }

inline SensorError set_frame_options(const CeptonSDKFrameOptions &options) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_set_frame_options(&options));
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
  return CEPTON_SDK_WRAP_C(cepton_sdk_listen_image_frames(cb, user_data));
}
inline SensorError unlisten_image_frames() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_unlisten_image_frames());
}

//------------------------------------------------------------------------------
// Sensors
//------------------------------------------------------------------------------
inline std::size_t get_n_sensors() { return cepton_sdk_get_n_sensors(); }

inline SensorError get_sensor_handle_by_serial_number(uint64_t serial_number,
                                                      SensorHandle &handle) {
  return CEPTON_SDK_WRAP_C(
      cepton_sdk_get_sensor_handle_by_serial_number(serial_number, &handle));
}

inline SensorError get_sensor_information_by_index(std::size_t idx,
                                                   SensorInformation &info) {
  return CEPTON_SDK_WRAP_C(
      cepton_sdk_get_sensor_information_by_index(idx, &info));
}
inline SensorError get_sensor_information(SensorHandle handle,
                                          SensorInformation &info) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_get_sensor_information(handle, &info));
}

//------------------------------------------------------------------------------
// Serial
//------------------------------------------------------------------------------
typedef void (*FpSerialReceiveCallback)(SensorHandle handle, const char *str,
                                        void *user_data);

inline SensorError listen_serial_lines(FpSerialReceiveCallback cb,
                                       void *const user_data) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_listen_serial_lines(cb, user_data));
}
inline SensorError unlisten_serial_lines() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_unlisten_serial_lines());
}

//------------------------------------------------------------------------------
// Networking
//------------------------------------------------------------------------------
typedef void (*FpNetworkReceiveCallback)(SensorHandle handle, int64_t timestamp,
                                         const uint8_t *buffer,
                                         size_t buffer_size, void *user_data);

inline SensorError listen_network_packets(FpNetworkReceiveCallback cb,
                                          void *const user_data) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_listen_network_packet(cb, user_data));
}
inline SensorError unlisten_network_packets() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_unlisten_network_packet());
}

inline SensorError mock_network_receive(SensorHandle handle, int64_t timestamp,
                                        const uint8_t *const buffer,
                                        std::size_t buffer_size) {
  return CEPTON_SDK_WRAP_C(
      cepton_sdk_mock_network_receive(handle, timestamp, buffer, buffer_size));
}
//------------------------------------------------------------------------------
// Capture Replay
//------------------------------------------------------------------------------
namespace capture_replay {
inline bool is_open() { return (bool)cepton_sdk_capture_replay_is_open(); }
inline SensorError open(const std::string &path) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_open(path.c_str()));
}
inline SensorError close() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_close());
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
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_seek(position));
}
/// Seek to relative capture file position [seconds].
/**
 * Returns error if position is invalid.
 */
inline SensorError seek_relative(float position) {
  position += get_position();
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_seek(position));
}

inline SensorError set_enable_loop(bool value) {
  return CEPTON_SDK_WRAP_C(
      cepton_sdk_capture_replay_set_enable_loop((int)value));
}
inline bool get_enable_loop() {
  return (bool)cepton_sdk_capture_replay_get_enable_loop();
}

inline SensorError set_speed(float speed) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_set_speed(speed));
}
inline float get_speed() { return cepton_sdk_capture_replay_get_speed(); }

inline SensorError resume_blocking_once() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_resume_blocking_once());
}
inline SensorError resume_blocking(float duration) {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_resume_blocking(duration));
}
inline bool is_running() { return cepton_sdk_capture_replay_is_running(); }
inline SensorError resume() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_resume());
}
inline SensorError pause() {
  return CEPTON_SDK_WRAP_C(cepton_sdk_capture_replay_pause());
}
}  // namespace capture_replay
}  // namespace cepton_sdk

#include "cepton_sdk_impl/cepton_sdk.inc"