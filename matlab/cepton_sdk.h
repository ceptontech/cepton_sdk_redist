/*
  Copyright Cepton Technologies Inc. 2017, All rights reserved.

  Cepton Sensor SDK C interface.
*/
#ifndef CEPTON_SDK_H
#define CEPTON_SDK_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "cepton_export.h"

#define CEPTON_SDK_VERSION 11

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
typedef int CeptonSensorErrorCode;

/// `CeptonSensorErrorCode` implementation.
enum _CeptonSensorErrorCode {
  CEPTON_SUCCESS = 0,
  CEPTON_ERROR_GENERIC = -1,
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  CEPTON_ERROR_COMMUNICATION = -6,  ///< Networking error
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  /// Invalid value or uninitialized struct
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  CEPTON_ERROR_NOT_INITIALIZED = -10,
  CEPTON_ERROR_INVALID_FILE_TYPE = -11,
  CEPTON_ERROR_FILE_IO = -12,
  CEPTON_ERROR_CORRUPT_FILE = -13,
  CEPTON_ERROR_NOT_OPEN = -14,
  CEPTON_ERROR_EOF = -15,

  CEPTON_FAULT_INTERNAL = -1000,  ///< Internal parameter out of range
  CEPTON_FAULT_EXTREME_TEMPERATURE = -1001,  ///< Reading exceed spec
  CEPTON_FAULT_EXTREME_HUMIDITY = -1002,     ///< Reading exceeds spec
  CEPTON_FAULT_EXTREME_ACCELERATION = -1003,
  CEPTON_FAULT_ABNORMAL_FOV = -1004,
  CEPTON_FAULT_ABNORMAL_FRAME_RATE = -1005,
  CEPTON_FAULT_MOTOR_MALFUNCTION = -1006,
  CEPTON_FAULT_LASER_MALFUNCTION = -1007,
  CEPTON_FAULT_DETECTOR_MALFUNCTION = -1008,
};

EXPORT const char *cepton_get_error_code_name(CeptonSensorErrorCode error_code);
EXPORT int cepton_is_error_code(CeptonSensorErrorCode error_code);
EXPORT int cepton_is_fault_code(CeptonSensorErrorCode error_code);

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef uint64_t CeptonSensorHandle;
static const CeptonSensorHandle CEPTON_NULL_HANDLE = 0LL;
static const CeptonSensorHandle CEPTON_SENSOR_HANDLE_FLAG_MOCK = 0x100000000LL;

typedef uint32_t CeptonSensorModel;

/// `CeptonSensorModel` implementation.
enum _CeptonSensorModel {
  HR80T = 1,
  HR80M = 2,
  HR80W = 3,
  SORA_200 = 4,
  VISTA_860 = 5,
  HR80T_R2 = 6,
};

struct EXPORT CeptonSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[28];
  CeptonSensorModel model;
  char firmware_version[32];

  float last_reported_temperature;  ///< [celsius]
  float last_reported_humidity;     ///< [%]
  float last_reported_age;          ///< [hours]
  float padding;

  uint64_t ptp_ts;

  uint8_t gps_ts_year;   ///< 0-99 (2017 -> 17)
  uint8_t gps_ts_month;  ///< 1-12
  uint8_t gps_ts_day;    ///< 1-31
  uint8_t gps_ts_hour;   ///< 0-23
  uint8_t gps_ts_min;    ///< 0-59
  uint8_t gps_ts_sec;    ///< 0-59

  uint8_t return_count;
  uint8_t reserved;

  // Flags
  uint32_t is_mocked : 1;          ///< Created by capture replay
  uint32_t is_pps_connected : 1;   ///< GPS PPS is available
  uint32_t is_nmea_connected : 1;  ///< GPS NMEA is available
  uint32_t is_ptp_connected : 1;   ///< PTP is available
  uint32_t is_calibrated : 1;
};
EXPORT extern const size_t cepton_sensor_information_size;

/// Point in image coordinates (focal length = 1).
/**
 * To convert to 3d point, refer to `cepton_sdk_util.hpp`.
*/
struct EXPORT CeptonSensorImagePoint {
  uint64_t timestamp;  ///< unix time [microseconds]
  float image_x;       ///< x image coordinate
  float distance;      ///< distance [meters]
  float image_z;       ///< z image coordinate
  float intensity;     ///< 0-1 scaled intensity
  uint8_t return_number; ///< 0=first return, 1=second return
  uint8_t valid;       ///< 1=valid; 0=clipped/invalid
  uint8_t saturated;   ///< If satruated, intensity cannot be trusted
  uint8_t reserved;
};
EXPORT extern const size_t cepton_sensor_image_point_size;

//------------------------------------------------------------------------------
// SDK Setup
//------------------------------------------------------------------------------
typedef uint32_t CeptonSDKControl;

/// `CeptonSDKControl` implementation.
enum _CeptonSDKControl {
  /// Disable networking operations.
  /**
   * Useful for running multiple instances of sdk in different processes.
   * Must pass packets manually to `cepton_sdk::mock_network_receive`.
   */
  CEPTON_SDK_CONTROL_DISABLE_NETWORK = 1 << 1,
  /// Disable marking image clipped points as invalid.
  /**
   * Does not affect number of points returned.
   */
  CEPTON_SDK_CONTROL_DISABLE_IMAGE_CLIP = 1 << 2,
  /// Disable marking distance clipped points as invalid.
  /**
   * Does not affect number of points returned.
   */
  CEPTON_SDK_CONTROL_DISABLE_DISTANCE_CLIP = 1 << 3,
  /// Enable multiple returns.
  /**
   * When set, `cepton_sdk::SensorInformation::return_count` will indicate the
   * number of returns per laser. Can only be set at sdk initialization.
   */
  CEPTON_SDK_CONTROL_ENABLE_MULTIPLE_RETURNS = 1 << 4,
};

typedef uint32_t CeptonSDKFrameMode;

/// `CeptonSDKFrameMode` implementation.
enum _CeptonSDKFrameMode {
  /// Report points immediately.
  /**
   * This is the default.
   */
  CEPTON_SDK_FRAME_STREAMING = 0,
  /// Report points at fixed time intervals.
  /**
   * Interval controlled by `CeptonSDKFrameOptions::frame_length`.
   */
  CEPTON_SDK_FRAME_TIMED = 1,
  /// Report points when the field of view is covered once.
  CEPTON_SDK_FRAME_COVER = 2,
  /// Report points when the scan pattern goes through a full cycle.
  CEPTON_SDK_FRAME_CYCLE = 3,

  CEPTON_SDK_FRAME_MODE_MAX = 3
};

struct EXPORT CeptonSDKFrameOptions {
  size_t signature;
  CeptonSDKFrameMode mode;
  float length;
};
EXPORT struct CeptonSDKFrameOptions cepton_sdk_create_frame_options();

struct EXPORT CeptonSDKOptions {
  size_t signature;
  CeptonSDKControl control_flags;
  struct CeptonSDKFrameOptions frame;
  uint16_t port;
};
EXPORT struct CeptonSDKOptions cepton_sdk_create_options();

typedef void (*FpCeptonSensorErrorCallback)(CeptonSensorHandle handle,
                                            CeptonSensorErrorCode error_code,
                                            const char *error_msg,
                                            const void *error_data,
                                            size_t error_data_size,
                                            void *user_data);

EXPORT CeptonSensorErrorCode cepton_sdk_is_initialized();
EXPORT CeptonSensorErrorCode
cepton_sdk_initialize(int ver, const struct CeptonSDKOptions *const options,
                      FpCeptonSensorErrorCallback cb, void *const user_data);
EXPORT CeptonSensorErrorCode cepton_sdk_deinitialize();

EXPORT CeptonSensorErrorCode
cepton_sdk_set_control_flags(CeptonSDKControl mask, CeptonSDKControl flags);
EXPORT CeptonSDKControl cepton_sdk_get_control_flags();
EXPORT int cepton_sdk_has_control_flag(CeptonSDKControl flag);

EXPORT uint16_t cepton_sdk_get_port();
EXPORT CeptonSensorErrorCode cepton_sdk_set_port(uint16_t port);

EXPORT CeptonSensorErrorCode
cepton_sdk_set_frame_options(const struct CeptonSDKFrameOptions *const options);
EXPORT CeptonSDKFrameMode cepton_sdk_get_frame_mode();
EXPORT float cepton_sdk_get_frame_length();

EXPORT CeptonSensorErrorCode cepton_sdk_clear_cache();

//------------------------------------------------------------------------------
// Points
//------------------------------------------------------------------------------
typedef void (*FpCeptonSensorImageDataCallback)(
    CeptonSensorHandle handle, size_t n_points,
    const struct CeptonSensorImagePoint *c_points, void *user_data);

EXPORT CeptonSensorErrorCode cepton_sdk_listen_image_frames(
    FpCeptonSensorImageDataCallback cb, void *const user_data);
EXPORT CeptonSensorErrorCode cepton_sdk_unlisten_image_frames();

//------------------------------------------------------------------------------
// Sensors
//------------------------------------------------------------------------------
EXPORT size_t cepton_sdk_get_n_sensors();
EXPORT CeptonSensorErrorCode cepton_sdk_get_sensor_handle_by_serial_number(
    uint64_t serial_number, CeptonSensorHandle *const handle);
EXPORT CeptonSensorErrorCode cepton_sdk_get_sensor_information_by_index(
    size_t idx, struct CeptonSensorInformation *const info);
EXPORT CeptonSensorErrorCode cepton_sdk_get_sensor_information(
    CeptonSensorHandle handle, struct CeptonSensorInformation *const info);

//------------------------------------------------------------------------------
// Networking
//------------------------------------------------------------------------------
typedef void (*FpCeptonNetworkReceiveCallback)(CeptonSensorHandle handle,
                                               uint8_t const *buffer,
                                               size_t buffer_size,
                                               void *user_data);

EXPORT CeptonSensorErrorCode cepton_sdk_listen_network_packet(
    FpCeptonNetworkReceiveCallback cb, void *const user_data);
EXPORT CeptonSensorErrorCode cepton_sdk_unlisten_network_packet();

EXPORT CeptonSensorErrorCode cepton_sdk_set_mock_time_base(uint64_t time_base);

EXPORT CeptonSensorErrorCode cepton_sdk_mock_network_receive(
    CeptonSensorHandle handle, const uint8_t *const buffer, size_t buffer_size);

//------------------------------------------------------------------------------
// Capture Replay
//------------------------------------------------------------------------------
EXPORT int cepton_sdk_capture_replay_is_open();
EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_open(const char *const path);
EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_close();

EXPORT uint64_t cepton_sdk_capture_replay_get_start_time();
EXPORT float cepton_sdk_capture_replay_get_position();
EXPORT float cepton_sdk_capture_replay_get_length();
EXPORT int cepton_sdk_capture_replay_is_end();
EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_rewind();
EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_seek(float position);

EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_set_enable_loop(int enable_loop);
EXPORT int cepton_sdk_capture_replay_get_enable_loop();

EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_set_speed(float speed);
EXPORT float cepton_sdk_capture_replay_get_speed();

EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_resume_blocking_once();
EXPORT CeptonSensorErrorCode
cepton_sdk_capture_replay_resume_blocking(float duration);
EXPORT int cepton_sdk_capture_replay_is_running();
EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_resume();
EXPORT CeptonSensorErrorCode cepton_sdk_capture_replay_pause();

#ifdef __cplusplus
}  // extern "C"
#endif

#undef EXPORT

#endif  // CEPTON_SDK_H
