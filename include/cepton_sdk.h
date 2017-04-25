//
// Copyright Cepton Technologies Inc. 2017, All rights reserved.
//
// Cepton Sensor SDK v0.5 (Beta)
//
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CEPTON_SDK_VERSION 5

typedef uint64_t CeptonSensorHandle;  // Handle of the sensor device

enum CeptonSensorErrorCode {
  CEPTON_SUCCESS = 0,
  CEPTON_ERROR_GENERIC = -1,
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  CEPTON_ERROR_TOO_MANY_SENSORS = -3,
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  CEPTON_ERROR_COMMUNICATION = -6, // Error communicating with the sensor
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  CEPTON_ERROR_NOT_INITIALIZED = -10,
};

enum CeptonSensorEvent {
  CEPTON_EVENT_ATTACH = 1,
  CEPTON_EVENT_DETACH = 2, // For now never fired
  CEPTON_EVENT_FRAME = 3,
};

struct CeptonSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[32];
  char firmware_version[32];

  float last_reported_temperature; // Celsius
  float last_reported_humidity; // %
  float last_reported_age; // hours

  // Note: GPS timestamp reported here is GMT time
  uint8_t gps_ts_year; // e.g. 2017 => 17
  uint8_t gps_ts_month; // 1-12
  uint8_t gps_ts_day; // 1-31
  uint8_t gps_ts_hour; // 0-23
  uint8_t gps_ts_min; // 0-59
  uint8_t gps_ts_sec; // 0-59

  uint8_t sensor_index; // Internal index, can be passed to _by_index functions

  // flags
  uint32_t is_mocked : 1; // Set if this device is created through cepton_sdk_mock_network_receive
  uint32_t is_pps_connected : 1; // Set if GPS/PPS is available
  uint32_t is_nmea_connected : 1; // Set if GPS/NMEA is available
};
//--------------------------------------------
// Global state/service management

typedef void (*FpCeptonSensorEventCallback)(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int sensor_event);

enum {
  CEPTON_SDK_CONTROL_RETURN_UNMEASURABLE = 1,
};

// initialize will allocate buffers, make connections, launch threads etc.
// Flag is a bit field defined by the enum above
int cepton_sdk_initialize(int ver, unsigned flags, FpCeptonSensorEventCallback cb);

// deallocate and disconnect
int cepton_sdk_deinitialize();

//--------------------------------------------
// Receiving data from sensor
struct CeptonSensorPoint {
  uint64_t timestamp;  // Microseconds from start of epoch
  float x, y, z;       // These measurements in meters
  float intensity;     // 0-1 range
};

typedef void (*FpCeptonSensorDataCallback)(int error_code, CeptonSensorHandle sensor,
  size_t n_points, struct CeptonSensorPoint const *p_points);

// Register callbacks that will be called once per "frame"
int cepton_sdk_listen_frames(FpCeptonSensorDataCallback cb);
int cepton_sdk_unlisten_frames(FpCeptonSensorDataCallback cb);

// Register calbacks that will be called once per "scan-line"
int cepton_sdk_listen_scanlines(FpCeptonSensorDataCallback cb);
int cepton_sdk_unlisten_scanlines(FpCeptonSensorDataCallback cb);

//--------------------------------------------
// Discover connected sensors
int cepton_sdk_get_number_of_sensors();

struct CeptonSensorInformation const *cepton_sdk_get_sensor_information(CeptonSensorHandle h);
struct CeptonSensorInformation const *cepton_sdk_get_sensor_information_by_index(int sensor_index);

//--------------------------------------------
struct CeptonSensorTransform {
  // We use quaternion to represent rotation, this must be normalized
  float rotation_quaternion[4]; // [Axis*sin(theta/2), cos(theta/2)]
  float translation[3]; // X, Y, Z, [m]
};

int cepton_sdk_set_transform(CeptonSensorHandle h, struct CeptonSensorTransform const *transform);
int cepton_sdk_get_transform(CeptonSensorHandle h, struct CeptonSensorTransform *transform);

//--------------------------------------------
// Mock Sensor replay and capture
void cepton_sdk_mock_network_receive(uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);

typedef void(*FpCeptonNetworkReceiveCb)(int error_code, uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);
int cepton_sdk_listen_network_packet(FpCeptonNetworkReceiveCb cb);

#ifdef __cplusplus
} // extern "C"
#endif
