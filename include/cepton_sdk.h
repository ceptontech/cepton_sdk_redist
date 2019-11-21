/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C interface.
*/
#ifndef CEPTON_SDK_H
#define CEPTON_SDK_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------
// CEPTON_SDK_EXPORT
#ifndef CEPTON_SDK_EXPORT
#ifdef CEPTON_SDK_COMPILING  // Export
#ifdef _MSC_VER
#define CEPTON_SDK_EXPORT __declspec(dllexport)
#elif __GNUC__
#define CEPTON_SDK_EXPORT __attribute__((visibility("default")))
#else
#define CEPTON_SDK_EXPORT
#endif
#elif defined(CEPTON_SDK_STATIC)  // Import static
#define CEPTON_SDK_EXPORT
#else  // Import shared
#ifdef _MSC_VER
#define CEPTON_SDK_EXPORT __declspec(dllimport)
#else
#define CEPTON_SDK_EXPORT
#endif
#endif
#endif

// CEPTON_DEPRECATED
#ifndef CEPTON_DEPRECATED
#if defined(_MSC_VER)
#define CEPTON_DEPRECATED __declspec(deprecated)
#elif defined(__GNUC__)
#define CEPTON_DEPRECATED __attribute__((deprecated))
#else
#define CEPTON_DEPRECATED
#endif
#endif

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
/// Error code returned by most library functions.
/**
 * Must call `cepton_sdk_get_error` if nonzero error code is returned.
 */
typedef int32_t CeptonSensorErrorCode;
enum _CeptonSensorErrorCode {
  /// No error.
  CEPTON_SUCCESS = 0,
  /// Generic error.
  CEPTON_ERROR_GENERIC = -1,
  /// Failed to allocate heap memory.
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  /// Could not find sensor.
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  /// SDK version mismatch.
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  /// Networking error.
  CEPTON_ERROR_COMMUNICATION = -6,
  /// Callback already set.
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  /// Invalid value or uninitialized struct.
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  /// Already initialized.
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  /// Not initialized.
  CEPTON_ERROR_NOT_INITIALIZED = -10,
  /// Invalid file type.
  CEPTON_ERROR_INVALID_FILE_TYPE = -11,
  /// File IO error.
  CEPTON_ERROR_FILE_IO = -12,
  /// Corrupt/invalid file.
  CEPTON_ERROR_CORRUPT_FILE = -13,
  /// Not open.
  CEPTON_ERROR_NOT_OPEN = -14,
  /// End of file.
  CEPTON_ERROR_EOF = -15,
  /// Functionality not supported by device
  CEPTON_ERROR_NOT_SUPPORTED = -16,
  /// Device response invalid
  CEPTON_ERROR_INVALID_RESPONSE = -17,

  /// Internal sensor parameter out of range.
  CEPTON_FAULT_INTERNAL = -1000,
  /// Extreme sensor temperature fault.
  CEPTON_FAULT_EXTREME_TEMPERATURE = -1001,
  /// Extreme sensor humidity fault.
  CEPTON_FAULT_EXTREME_HUMIDITY = -1002,
  /// Extreme sensor acceleration fault.
  CEPTON_FAULT_EXTREME_ACCELERATION = -1003,
  /// Abnormal sensor FOV fault.
  CEPTON_FAULT_ABNORMAL_FOV = -1004,
  /// Abnormal sensor frame rate fault.
  CEPTON_FAULT_ABNORMAL_FRAME_RATE = -1005,
  /// Sensor motor malfunction fault.
  CEPTON_FAULT_MOTOR_MALFUNCTION = -1006,
  /// Sensor laser malfunction fault.
  CEPTON_FAULT_LASER_MALFUNCTION = -1007,
  /// Sensor detector malfunction fault.
  CEPTON_FAULT_DETECTOR_MALFUNCTION = -1008,
};

/// Returns error code name string.
/**
 * Returns empty string if error code is invalid.
 *
 * @return Error code name string. Owned by SDK. Valid until next SDK call in
 * current thread.
 */
CEPTON_SDK_EXPORT const char *cepton_get_error_code_name(
    CeptonSensorErrorCode error_code);
/// Returns whether `error_code` is of the form `CEPTON_ERROR_*`.
CEPTON_SDK_EXPORT int cepton_is_error_code(CeptonSensorErrorCode error_code);
/// Returns whether `error_code` is of the form `CEPTON_FAULT_*`.
CEPTON_SDK_EXPORT int cepton_is_fault_code(CeptonSensorErrorCode error_code);

/// Returns and clears last sdk error.
/**
 * @param error_msg Returned error message string. Owned by the SDK, and valid
 * until next SDK call in the current thread.
 * @return Error code.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_get_error(const char **error_msg);

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
/// Sensor identifier.
/**
 * Generated from sensor IP address.
 */
typedef uint64_t CeptonSensorHandle;
/// Internal use only.
static const CeptonSensorHandle CEPTON_NULL_HANDLE = 0LL;
/// Internal use only.
static const CeptonSensorHandle CEPTON_SENSOR_HANDLE_FLAG_MOCK = 0x100000000LL;

/// Sensor model.
typedef uint16_t CeptonSensorModel;
enum _CeptonSensorModel {
  HR80W = 3,
  HR80T_R2 = 6,
  VISTA_860_GEN2 = 7,
  VISTA_X120 = 10,
  SORA_P60 = 11,
  VISTA_P60 = 12,
  VISTA_X15 = 13,
  VISTA_P90 = 14,
  SORA_P90 = 15,
  VISTA_P61 = 16,
  SORA_P61 = 17,
  // 18 is reserved for VISTA_H
  // 19 is reserved for VISTA_P60 Rev2 firmware releases
  CEPTON_SENSOR_MODEL_MAX = 18,
};

/// Returns whether sensor model is of the form `SORA_*`.
CEPTON_SDK_EXPORT int cepton_is_sora(CeptonSensorModel model);
/// Returns whether sensor model is of the form `HR80_*`.
CEPTON_SDK_EXPORT int cepton_is_hr80(CeptonSensorModel model);
/// Returns whether sensor model is of the form `VISTA_*`.
CEPTON_SDK_EXPORT int cepton_is_vista(CeptonSensorModel model);

/// Sensor information struct.
/**
 * Returned by `cepton_sdk_get_sensor_information*`.
 */
struct CEPTON_SDK_EXPORT CeptonSensorInformation {
  /// Sensor identifier (generated from IP address).
  CeptonSensorHandle handle;
  /// Sensor serial number.
  uint64_t serial_number;
  /// Full sensor model name.
  char model_name[28];
  /// Sensor model.
  CeptonSensorModel model;
  uint16_t reserved;
  /// Firmware version string.
  char firmware_version[28];

#ifdef CEPTON_SIMPLE
  /// Firmware version struct.
  uint32_t formal_firmware_version;
#else
  /// Firmware version struct.
  struct {
    /// Major firmware version.
    uint8_t major;
    /// Minor firmware version.
    uint8_t minor;
    uint8_t unused[2];
  } formal_firmware_version;
#endif

  /// [celsius].
  float last_reported_temperature;
  /// [%].
  float last_reported_humidity;
  /// [hours].
  float last_reported_age;

  /// Time between measurements [seconds].
  float measurement_period;

  /// PTP time [microseconds].
  int64_t ptp_ts;

  // GPS time parsed from NMEA sentence.
  uint8_t gps_ts_year;   ///< (0-99) (e.g. 2017 -> 17)
  uint8_t gps_ts_month;  ///< (1-12)
  uint8_t gps_ts_day;    ///< (1-31)
  uint8_t gps_ts_hour;   ///< (0-23)
  uint8_t gps_ts_min;    ///< (0-59)
  uint8_t gps_ts_sec;    ///< (0-59)

  /// Number of returns per measurement.
  uint8_t return_count;
  /// Number of image segments.
  uint8_t segment_count;

#ifdef CEPTON_SIMPLE
  /// Bit flags.
  uint32_t flags;
#else
  union {
    /// Bit flags.
    uint32_t flags;
    struct {
      /// Created by capture replay.
      uint32_t is_mocked : 1;
      /// GPS PPS is available.
      uint32_t is_pps_connected : 1;
      /// GPS NMEA is available.
      uint32_t is_nmea_connected : 1;
      /// PTP is available.
      uint32_t is_ptp_connected : 1;
      /// Calibration loaded.
      uint32_t is_calibrated : 1;
      /// Hit temperature limit.
      uint32_t is_over_heated : 1;
      /// Sync fire enabled (disabled by default).
      uint32_t is_sync_firing_enabled : 1;
    };
  };
#endif
};
/// Internal use only.
CEPTON_SDK_EXPORT extern const size_t cepton_sensor_information_size;

/// Measurement return type flags (for multi-return).
typedef uint8_t CeptonSensorReturnType;
enum _CeptonSensorReturnType {
  /// Highest intensity return.
  CEPTON_RETURN_STRONGEST = 1 << 0,
  /// Farthest return.
  CEPTON_RETURN_FARTHEST = 1 << 1,
};

/// Point in pinhole image coordinates (focal length = 1).
/**
 * To convert to 3d point, use
 * `cepton_sdk::util::convert_sensor_image_point_to_point`.
 */
struct CEPTON_SDK_EXPORT CeptonSensorImagePoint {
  /// Unix time [microseconds].
  int64_t timestamp;
  /// x image coordinate.
  float image_x;
  /// Distance [meters].
  float distance;
  /// z image coordinate.
  float image_z;
  /// Diffuse reflectance (normal: [0-1], retroreflective: >1).
  float intensity;
  /// Return type flags.
  CeptonSensorReturnType return_type;

#ifdef CEPTON_SIMPLE
  /// Bit flags.
  uint8_t flags;
#else
  union {
    /// Bit flags.
    uint8_t flags;
    struct {
      /// If `false`, then `distance` and `intensity` are invalid.
      uint8_t valid : 1;
      /// If `true`, then `intensity` is invalid, and the `distance` is
      /// innacurate.
      uint8_t saturated : 1;
    };
  };
#endif

  uint8_t segment_id;
  uint8_t reserved[1];
};
/// Internal use only.
CEPTON_SDK_EXPORT extern const size_t cepton_sensor_image_point_size;

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
// Limits for preallocating buffers.
// These numbers are guaranteed to be safe for 6 months from SDK release.
#define CEPTON_SDK_MAX_POINTS_PER_PACKET 400
#define CEPTON_SDK_MAX_POINTS_PER_FRAME 50000
#define CEPTON_SDK_MAX_POINTS_PER_SECOND 1000000
#define CEPTON_SDK_MAX_FRAMES_PER_SECOND 40

//------------------------------------------------------------------------------
// SDK Setup
//------------------------------------------------------------------------------
/// API version.
/**
 * Used in `cepton_sdk_initialize` to validate shared library API.
 */
#define CEPTON_SDK_VERSION 19

/// Returns library version string.
/**
 * This is different from `CEPTON_SDK_VERSION`.
 *
 * @return Version string. Owned by SDK. Valid until next SDK call in current
 * thread.
 */
CEPTON_SDK_EXPORT const char *cepton_sdk_get_version_string();
/// Returns library version major.
CEPTON_SDK_EXPORT int cepton_sdk_get_version_major();
/// Returns library version minor.
CEPTON_SDK_EXPORT int cepton_sdk_get_version_minor();
/// Returns library version patch.
CEPTON_SDK_EXPORT int cepton_sdk_get_version_patch();

/// SDK setup flags.
typedef uint32_t CeptonSDKControl;
enum _CeptonSDKControl {
  /// Disable networking operations.
  /**
   * Useful for running multiple instances of sdk in different processes.
   * Must pass packets manually to `cepton_sdk::mock_network_receive`.
   */
  CEPTON_SDK_CONTROL_DISABLE_NETWORK = 1 << 1,
  /// Enable multiple returns.
  /**
   * When set, `cepton_sdk::SensorInformation::return_count` will indicate the
   * number of returns per laser. 
   * Can only be set at SDK initialization.
   */
  CEPTON_SDK_CONTROL_ENABLE_MULTIPLE_RETURNS = 1 << 4,
  /// Always use packet timestamps (disable GPS/PTP timestamps).
  CEPTON_SDK_CONTROL_HOST_TIMESTAMPS = 1 << 6,
  CEPTON_SDK_CONTROL_RESERVED = 1 << 7,
};

/// Controls frequency of points being reported.
typedef uint32_t CeptonSDKFrameMode;
enum _CeptonSDKFrameMode {
  /// Report points by packet.
  CEPTON_SDK_FRAME_STREAMING = 0,
  /// Report points at fixed time intervals.
  /**
   * Interval controlled by `CeptonSDKFrameOptions::length`.
   */
  CEPTON_SDK_FRAME_TIMED = 1,
  /// Report points when the field of view is covered once.
  /**
   * Use this for a fast frame rate.
   * - For Sora series, detects scanline (left-to-right or right-to-left).
   * - For HR80 series, detects half scan cycle (left-to-right or
   * right-to-left).
   * - For Vista series, detects half scan cycle.
   */
  CEPTON_SDK_FRAME_COVER = 2,
  /// Report points when the scan pattern goes through a full cycle.
  /**
   * Use this for a consistent, repeating frame.
   * Typically 2x longer frame than `CEPTON_SDK_FRAME_COVER` mode.
   */
  CEPTON_SDK_FRAME_CYCLE = 3,

  CEPTON_SDK_FRAME_MODE_MAX = 3
};

/// SDK frame options.
/**
 * Must use `cepton_sdk_create_frame_options` to create.
 */
struct CEPTON_SDK_EXPORT CeptonSDKFrameOptions {
  /// Internal use only.
  size_t signature;
  /// Default: `CEPTON_SDK_FRAME_STREAMING`.
  CeptonSDKFrameMode mode;
  /// Frame length [seconds].
  /**
   * Default: 0.05.
   * Only used if `mode`=`CEPTON_SDK_FRAME_TIMED`.
   */
  float length;
};
/// Create frame options.
CEPTON_SDK_EXPORT struct CeptonSDKFrameOptions
cepton_sdk_create_frame_options();

/// SDK initialization options.
/**
 * Must call `cepton_sdk_create_options` to create.
 */
struct CEPTON_SDK_EXPORT CeptonSDKOptions {
  /// Internal use only.
  size_t signature;
  /// Default: 0.
  CeptonSDKControl control_flags;
  struct CeptonSDKFrameOptions frame;
  /// Network listen port. Default: 8808.
  uint16_t port;
};
/// Create SDK options.
CEPTON_SDK_EXPORT struct CeptonSDKOptions cepton_sdk_create_options();

/// Callback for receiving sdk and sensor errors.
/**
 * @param handle Associated sensor handle.
 * @param error_code Error code.
 * @param error_msg Error message string. Owned by SDK.
 * @param error_data Not implemented.
 * @param error_data_size Not implemented.
 * @param user_data User instance pointer.
 */
typedef void (*FpCeptonSensorErrorCallback)(CeptonSensorHandle handle,
                                            CeptonSensorErrorCode error_code,
                                            const char *error_msg,
                                            const void *error_data,
                                            size_t error_data_size,
                                            void *user_data);

/// Returns true if sdk is initialized.
CEPTON_SDK_EXPORT int cepton_sdk_is_initialized();
/// Initializes settings and networking.
/**
 * Must be called before any other sdk function listed below.
 *
 * @param ver `CEPTON_SDK_VERSION`
 * @param options SDK options.
 * @param cb Error callback.
 * @param user_data Error callback user instance pointer.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_initialize(int ver, const struct CeptonSDKOptions *const options,
                      FpCeptonSensorErrorCallback cb, void *const user_data);

/// Resets everything and deallocates memory.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_deinitialize();

/// Sets SDK control flags.
/**
 * @param mask Bit mask for selecting flags to change.
 * @param flags Bit flag values.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_set_control_flags(CeptonSDKControl mask, CeptonSDKControl flags);
/// Returns SDK control flag.
CEPTON_SDK_EXPORT CeptonSDKControl cepton_sdk_get_control_flags();
/// Returns whether SDK control flag is set.
CEPTON_SDK_EXPORT int cepton_sdk_has_control_flag(CeptonSDKControl flag);

/// Clears sensors.
/**
 * Use when loading/unloading capture file.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_clear();

/// Sets network listen port.
/**
 * Default: 8808.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_set_port(uint16_t port);

/// Returns network listen port.
CEPTON_SDK_EXPORT uint16_t cepton_sdk_get_port();

/// Sets frame options.
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_set_frame_options(const struct CeptonSDKFrameOptions *const options);

/// Returns frame mode.
CEPTON_SDK_EXPORT CeptonSDKFrameMode cepton_sdk_get_frame_mode();

/// Returns frame length.
CEPTON_SDK_EXPORT float cepton_sdk_get_frame_length();

//------------------------------------------------------------------------------
// Points
//------------------------------------------------------------------------------
/// Callback for receiving image points.
/**
 * Set the frame options to control the callback rate.
 * @param handle Sensor handle.
 * @param n_points Points array size.
 * @param c_points Points array. Owned by SDK.
 * @param user_data User instance pointer.
 */
typedef void (*FpCeptonSensorImageDataCallback)(
    CeptonSensorHandle handle, size_t n_points,
    const struct CeptonSensorImagePoint *c_points, void *user_data);

/// Sets image frame callback.
/**
 * Returns points at frequency specified by `cepton_sdk::FrameOptions::mode`.
 * Each frame contains all possible points (use
 * `cepton_sdk::SensorImagePoint::valid` to filter points). Points are ordered
 * by measurement, segment, and return:
 *
 * ```
 * measurement_count = n_points / (segment_count * return_count)
 * idx = ((i_measurement) * segment_count + i_segment) * return_count + i_return
 * ```
 *
 * Returns error if callback already registered.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_listen_image_frames(
    FpCeptonSensorImageDataCallback cb, void *const user_data);

/// Clears image frame callback.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_unlisten_image_frames();

//------------------------------------------------------------------------------
// Sensors
//------------------------------------------------------------------------------
/**
 * Get number of sensors attached.
 * Use to check for new sensors. Sensors are not deleted until deinitialization.
 */
CEPTON_SDK_EXPORT size_t cepton_sdk_get_n_sensors();
/// Looks up sensor handle by serial number.
/**
 * Returns error if sensor not found.
 *
 * @param serial_number Sensor serial number.
 * @param handle Sensor handle.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_get_sensor_handle_by_serial_number(uint64_t serial_number,
                                              CeptonSensorHandle *const handle);
/// Returns sensor information by sensor index.
/**
 * Useful for getting information for all sensors.
 * Valid indices are in range [0, `cepton_sdk_get_n_sensors()`).
 *
 * Returns error if index invalid.
 *
 * @param idx Sensor index. Returns error if invalid.
 * @param info Sensor information.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_get_sensor_information_by_index(
    size_t idx, struct CeptonSensorInformation *const info);
/// Returns sensor information by sensor handle.
/**
 * @param handle Sensor handle. Returns error if invalid.
 * @param info Sensor information.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_get_sensor_information(
    CeptonSensorHandle handle, struct CeptonSensorInformation *const info);

//------------------------------------------------------------------------------
// Serial
//------------------------------------------------------------------------------
/// Callback for receiving serial data (e.g. NMEA).
/**
 * @param handle Sensor handle.
 * @param str Serial line string. Owned by SDK.
 * @param user_data User instance pointer.
 */
typedef void (*FpCeptonSerialReceiveCallback)(CeptonSensorHandle handle,
                                              const char *str, void *user_data);

/// Sets serial line callback.
/**
 * Useful for listening to NMEA data from GPS attached to sensor.
 * Each callback contains 1 line of serial data (including newline characters).
 *
 * Returns error if callback already registered.
 *
 * @param cb Callback.
 * @param user_data User instance pointer.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_listen_serial_lines(
    FpCeptonSerialReceiveCallback cb, void *const user_data);

/// Clears serial line callback.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_unlisten_serial_lines();

//------------------------------------------------------------------------------
// Networking
//------------------------------------------------------------------------------
/// Callback for receiving network packets.
/**
 * Returns error if callback already set.
 *
 * @param handle Sensor handle.
 * @param timestamp Packet Unix timestamp [microseconds].
 * @param buffer Packet bytes.
 * @param buffer_size Buffer size.
 * @param user_data User instance pointer.
 */
typedef void (*FpCeptonNetworkReceiveCallback)(CeptonSensorHandle handle,
                                               int64_t timestamp,
                                               const uint8_t *buffer,
                                               size_t buffer_size,
                                               void *user_data);

/// Sets network packets callback.
/**
 * For internal use.
 *
 * Returns error if callback already registered.
 *
 * @param cb Callback.
 * @param user_data User instance pointer.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_listen_network_packet(
    FpCeptonNetworkReceiveCallback cb, void *const user_data);

/// Clears network packet callback.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_unlisten_network_packet();

/// Manually passes packets to sdk.
/**
 * Blocks while processing, and calls listener callbacks synchronously before
 * returning.
 *
 * @param handle Sensor handle.
 * @param timestamp Unix timestamp [microseconds].
 * @param buffer Packet bytes.
 * @param buffer_size Packet size.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_mock_network_receive(
    CeptonSensorHandle handle, int64_t timestamp, const uint8_t *const buffer,
    size_t buffer_size);

//------------------------------------------------------------------------------
// Capture Replay
//------------------------------------------------------------------------------
/// Returns whether capture replay is open.
CEPTON_SDK_EXPORT int cepton_sdk_capture_replay_is_open();
/// Opens capture replay.
/**
 * Must be called before any other replay functions listed below.
 *
 * @param path Path to PCAP capture file.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_open(const char *const path);

/// Closes capture replay.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_close();

/// Returns capture replay file name.
CEPTON_SDK_EXPORT const char *cepton_sdk_capture_replay_get_filename();

/// Returns capture start Unix timestamp [microseconds].
CEPTON_SDK_EXPORT int64_t cepton_sdk_capture_replay_get_start_time();
/// Returns capture file position [seconds].
CEPTON_SDK_EXPORT float cepton_sdk_capture_replay_get_position();
/// Returns capture file length [seconds].
CEPTON_SDK_EXPORT float cepton_sdk_capture_replay_get_length();
/// Returns whether at end of capture file.
/**
 * This is only relevant when using `resume_blocking` methods.
 */
CEPTON_SDK_EXPORT int cepton_sdk_capture_replay_is_end();
/// Rewinds capture replay to beginning.
/**
 * DEPRECATED: use `cepton_sdk_capture_replay_seek(0)`.
 */
CEPTON_DEPRECATED CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_rewind();
/// Seek to capture file position [seconds].
/**
 * @param position Seek position in range [0.0, capture length).
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_seek(float position);

/// Sets capture replay looping.
/**
 * If enabled, replay will automatically rewind at end.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_set_enable_loop(int enable_loop);

/// Returns whether capture replay looping is enabled.
CEPTON_SDK_EXPORT int cepton_sdk_capture_replay_get_enable_loop();

/// Sets speed multiplier for asynchronous replay.
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_set_speed(float speed);

/// Returns capture replay speed.
CEPTON_SDK_EXPORT float cepton_sdk_capture_replay_get_speed();

/// Replay next packet in current thread without sleeping.
/**
 * Pauses replay thread if running.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_resume_blocking_once();
/// Replay multiple packets synchronously.
/**
 * No sleep between packets. Pauses replay thread if running.
 *
 * @param duration Duration to replay. Must be non-negative.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_resume_blocking(float duration);
/// Returns true if replay thread is running.
CEPTON_SDK_EXPORT int cepton_sdk_capture_replay_is_running();
/// Resumes asynchronous replay thread.
/**
 * Packets are replayed in realtime. Replay thread sleeps in between packets.
 */
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_resume();
/// Pauses asynchronous replay thread.
CEPTON_SDK_EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_pause();

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CEPTON_SDK_H
