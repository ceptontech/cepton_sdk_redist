//
// Copyright Cepton Technologies Inc. 2017, All rights reserved.
//
// Cepton Sensor SDK v0.6d (Beta)
//
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef IS_WINDOWS
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CEPTON_SDK_VERSION 6

typedef uint64_t CeptonSensorHandle;  // Handle of the sensor device

enum CeptonSensorErrorCode {
  CEPTON_SUCCESS = 0,
  CEPTON_ERROR_GENERIC = -1,
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  CEPTON_ERROR_TOO_MANY_SENSORS = -3,
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  CEPTON_ERROR_COMMUNICATION = -6,  // Error communicating with the sensor
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  CEPTON_ERROR_NOT_INITIALIZED = -10,
};

const char *const cepton_get_error_code_name(int error_code);

enum CeptonSensorEvent {
  CEPTON_EVENT_ATTACH = 1,
  CEPTON_EVENT_DETACH = 2,  // For now never fired
  CEPTON_EVENT_FRAME = 3,
};

struct DLL_EXPORT CeptonSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[32];
  char firmware_version[32];

  float last_reported_temperature;  // Celsius
  float last_reported_humidity;     // %
  float last_reported_age;          // hours

  // Note: GPS timestamp reported here is GMT time
  uint8_t gps_ts_year;   // e.g. 2017 => 17
  uint8_t gps_ts_month;  // 1-12
  uint8_t gps_ts_day;    // 1-31
  uint8_t gps_ts_hour;   // 0-23
  uint8_t gps_ts_min;    // 0-59
  uint8_t gps_ts_sec;    // 0-59

  uint8_t sensor_index;  // Internal index, can be passed to _by_index functions

  // flags
  uint32_t is_mocked : 1;          // Set if this device is created through
                                   // cepton_sdk_mock_network_receive
  uint32_t is_pps_connected : 1;   // Set if GPS/PPS is available
  uint32_t is_nmea_connected : 1;  // Set if GPS/NMEA is available
};
DLL_EXPORT extern const size_t cepton_sensor_information_size;
//--------------------------------------------
// Global state/service management

typedef void (*FpCeptonSensorEventCallback)(
    int error_code, CeptonSensorHandle sensor,
    struct CeptonSensorInformation const *p_info, int sensor_event);

enum {
  CEPTON_SDK_CONTROL_FLAGS_RESERVED = 1,
};

// initialize will allocate buffers, make connections, launch threads etc.
// Flag is a bit field defined by the enum above
DLL_EXPORT int cepton_sdk_initialize(int ver, unsigned flags,
                                     FpCeptonSensorEventCallback cb);

// deallocate and disconnect
DLL_EXPORT int cepton_sdk_deinitialize();

//--------------------------------------------
// Receiving data from sensor
struct DLL_EXPORT CeptonSensorPoint {
  uint64_t timestamp;  // Microseconds from start of epoch
  float x, y, z;       // These measurements in meters
  float intensity;     // 0-1 range
};
DLL_EXPORT extern const size_t cepton_sensor_point_size;

DLL_EXPORT void cepton_sdk_get_sensor_points_timestamp(
    size_t n_points, struct CeptonSensorPoint const *p_points,
    uint64_t *const timestamp);
DLL_EXPORT void cepton_sdk_get_sensor_points_x(
    size_t n_points, struct CeptonSensorPoint const *p_points, float *const x);
DLL_EXPORT void cepton_sdk_get_sensor_points_y(
    size_t n_points, struct CeptonSensorPoint const *p_points, float *const y);
DLL_EXPORT void cepton_sdk_get_sensor_points_z(
    size_t n_points, struct CeptonSensorPoint const *p_points, float *const z);
DLL_EXPORT void cepton_sdk_get_sensor_points_intensity(
    size_t n_points, struct CeptonSensorPoint const *p_points,
    float *const intensity);

typedef void (*FpCeptonSensorDataCallback)(
    int error_code, CeptonSensorHandle sensor, size_t n_points,
    struct CeptonSensorPoint const *p_points);

// Register callbacks that will be called once per "frame"
DLL_EXPORT int cepton_sdk_listen_frames(FpCeptonSensorDataCallback cb);
DLL_EXPORT int cepton_sdk_unlisten_frames(FpCeptonSensorDataCallback cb);

// Register calbacks that will be called once per "scan-line"
DLL_EXPORT int cepton_sdk_listen_scanlines(FpCeptonSensorDataCallback cb);
DLL_EXPORT int cepton_sdk_unlisten_scanlines(FpCeptonSensorDataCallback cb);

struct DLL_EXPORT CeptonSensorImagePoint {
  uint64_t timestamp;  // Microseconds from start of epoch
  float image_x;       // x-offset on a unit image plane
  float distance;      // Distance in meters
  float image_z;       // z-offset on a unit image plane
  float intensity;     // 0-1 scaled intensity
};
DLL_EXPORT extern const size_t cepton_sensor_image_point_size;

DLL_EXPORT void cepton_sdk_get_sensor_image_points_timestamp(
    size_t n_points, struct CeptonSensorImagePoint const *p_points,
    uint64_t *const timestamp);
DLL_EXPORT void cepton_sdk_get_sensor_image_points_image_x(
    size_t n_points, struct CeptonSensorImagePoint const *p_points,
    float *const image_x);
DLL_EXPORT void cepton_sdk_get_sensor_image_points_distance(
    size_t n_points, struct CeptonSensorImagePoint const *p_points,
    float *const distance);
DLL_EXPORT void cepton_sdk_get_sensor_image_points_image_z(
    size_t n_points, struct CeptonSensorImagePoint const *p_points,
    float *const image_z);
DLL_EXPORT void cepton_sdk_get_sensor_image_points_intensity(
    size_t n_points, struct CeptonSensorImagePoint const *p_points,
    float *const intensity);

typedef void (*FpCeptonSensorImageDataCallback)(
    int error_code, CeptonSensorHandle sensor, size_t n_points,
    struct CeptonSensorImagePoint const *p_points);

// Register callbacks that will be called once per "frame"
DLL_EXPORT int cepton_sdk_listen_image_frames(
    FpCeptonSensorImageDataCallback cb);
DLL_EXPORT int cepton_sdk_unlisten_image_frames(
    FpCeptonSensorImageDataCallback cb);

// Register calbacks that will be called once per "scan-line"
DLL_EXPORT int cepton_sdk_listen_image_scanlines(
    FpCeptonSensorImageDataCallback cb);
DLL_EXPORT int cepton_sdk_unlisten_image_scanlines(
    FpCeptonSensorImageDataCallback cb);

//--------------------------------------------
// Discover connected sensors
DLL_EXPORT int cepton_sdk_get_number_of_sensors();

DLL_EXPORT int cepton_sdk_get_sensor_handle_by_serial_number(
    uint64_t serial_number, CeptonSensorHandle h);

DLL_EXPORT struct CeptonSensorInformation const *
cepton_sdk_get_sensor_information(CeptonSensorHandle h);
DLL_EXPORT struct CeptonSensorInformation const *
cepton_sdk_get_sensor_information_by_index(int sensor_index);

//--------------------------------------------
struct DLL_EXPORT CeptonSensorTransform {
  // We use quaternion to represent rotation, this must be normalized
  float rotation_quaternion[4];  // [Axis*sin(theta/2), cos(theta/2)]
  float translation[3];          // X, Y, Z, [m]
};
DLL_EXPORT extern const size_t cepton_sensor_transform_size;

DLL_EXPORT int cepton_sdk_set_transform(
    CeptonSensorHandle h, struct CeptonSensorTransform const *transform);
DLL_EXPORT int cepton_sdk_get_transform(
    CeptonSensorHandle h, struct CeptonSensorTransform *transform);

//--------------------------------------------
// Mock Sensor replay and capture

/*
  DEPRICATED. Use capture replay interface.
*/
DLL_EXPORT void cepton_sdk_mock_network_receive(uint64_t ipv4_address,
                                                uint8_t const *mac,
                                                uint8_t const *buffer,
                                                size_t size);

typedef void (*FpCeptonNetworkReceiveCb)(int error_code, uint64_t ipv4_address,
                                         uint8_t const *mac,
                                         uint8_t const *buffer, size_t size);
DLL_EXPORT int cepton_sdk_listen_network_packet(FpCeptonNetworkReceiveCb cb);
DLL_EXPORT int cepton_sdk_unlisten_network_packet(FpCeptonNetworkReceiveCb cb);

DLL_EXPORT int cepton_sdk_capture_replay_is_open(int *is_open_ptr);
DLL_EXPORT int cepton_sdk_capture_replay_open(char const *const path);
DLL_EXPORT int cepton_sdk_capture_replay_close();

/*
  Get capture file position (seconds relative to start of file).
*/
DLL_EXPORT int cepton_sdk_capture_replay_get_position(float *sec_ptr);

/*
  Get capture file length in seconds.
*/
DLL_EXPORT int cepton_sdk_capture_replay_get_length(float *sec_ptr);

/*
  Returns true if at end of capture file.
  This is only relevant when using resume_blocking methods.
*/
DLL_EXPORT int cepton_sdk_capture_replay_is_end(int *is_end_ptr);

/*
  Seek to start of capture file.

  Returns error if replay is running or is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_rewind();

/*
  Seek to capture file position.
  Position must be in range [0.0, capture length).

  Returns error if replay is running or is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_seek(float sec);

/*
  Replay next packet in current thread without sleeping.
  Pauses replay thread if it is running.
  Stops at end of file; must call rewind to continue.

  Returns error if replay is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_resume_blocking_once();

/*
  Replay multiple packets in current thread without sleeping between packets.
  Resume duration must be non-negative. Pauses replay thread if it is running.
  Stops at end of file; must call rewind to continue.

  Returns error if replay is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_resume_blocking(float sec);

/*
  Return true if replay thread is resumed, false if it is paused.
*/
DLL_EXPORT int cepton_sdk_capture_replay_is_running(int *is_running_ptr);

/*
  Resume asynchronous replay thread. Packets are replayed in realtime.
  Replay thread sleeps in between packets.

  Returns error if replay is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_resume(int enable_loop);

/*
  Pause asynchronous replay thread.

  Returns error if replay is not open.
*/
DLL_EXPORT int cepton_sdk_capture_replay_pause();

#ifdef __cplusplus
}  // extern "C"
#endif
