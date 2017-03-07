//
// Copyright Cepton Technologies Inc. 2017, All rights reserved.
//
// Cepton Sensor SDK v0.3 (Beta)
//
#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CEPTON_SDK_VERSION 3

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
};

enum CeptonSensorEvent {
  CEPTON_EVENT_ATTACH = 1,
  CEPTON_EVENT_DETACH = 2, // For now never fired
  CEPTON_EVENT_FRAME = 3,
};

#define CEPTON_SDK_CALIBRATION_SIGNATURE 0xB8435343
#define cepton_sdk_max_lasers_per_lidar 8

struct CeptonLaserCalibration {
  int16_t x_offset;
  int16_t y_offset;
  int16_t distance_offset;
  
  int16_t x_read_min;
  int16_t x_read_max;
  int16_t y_read_min;
  int16_t y_read_max;
};

struct CeptonSensorCalibration {
  uint32_t signature;
  // Intrinsic calibrations
  int16_t x_offset;
  int16_t y_offset;
  int16_t distance_offset;
  float distance_offset_poly_xx;
  float distance_offset_poly_yy;
  
  float x_scale;
  float y_scale;
  float distance_scale;
  float distance_scale_poly_xx;
  float distance_scale_poly_yy;
  
  float focal_length;
  
  struct CeptonLaserCalibration laser_calibrations[cepton_sdk_max_lasers_per_lidar];

  // Extrinsic calibrations
  // World (car) centric calibration, 6 degrees of freedom based on installation
  float lidar_x;    // In meters, relative to "ground"
  float lidar_y;   // In meters, positive is "ahead"
  float lidar_z;  // In meters, poistive is "right" when looking at
                         // direction of motion

  float lidar_inclination;  // In rads, 0 means parallel to round
  float lidar_azimuth;      // in rads, 0 means facing front.
  float lidar_tilt;         // in rads, 0 means top-side "up"

  // Internal derived rotation matrix
  float m00, m01, m02, m10, m11, m12, m20, m21, m22;

  // Internal device information
  uint16_t min_depth_cutoff;
};

struct CeptonSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[32];
  char firmware_version[32];

  float last_reported_temperature; // Celsius
  float last_reported_humidity; // %
  float last_reported_frequency;
  float last_reported_age; // hours

  // Internal data, these will change over time, please don't depend on them
  struct CeptonSensorCalibration calibration;
  uint64_t timestamp_offset;

  // Internal flags
  uint32_t is_mocked : 1; // Set if this device is created through cepton_sdk_mock_network_receive
  uint32_t unused_flags : 31;
};
//--------------------------------------------
// Global state/service management

typedef void (*FpCeptonSensorEventCallback)(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int sensor_event);

// initialize will allocate buffers, make connections, launch threads etc.
// NOTE: flags is reserved and must be 0 for now.
int cepton_sdk_initialize(int ver, unsigned flags, FpCeptonSensorEventCallback cb);

// deallocate and disconnect
int cepton_sdk_deinitialize();

//--------------------------------------------
// Receiving data from sensor
struct CeptonSensorPoint {
  uint64_t timestamp;  // Microseconds since last successful cepton_sdk_initialize()
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
// Sensor calibrations
int cepton_sdk_set_calibration(CeptonSensorHandle h,
  struct CeptonSensorCalibration const *cal);

//--------------------------------------------
// Mock Sensor replay and capture
void cepton_sdk_mock_network_receive(uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);

typedef void(*FpCeptonNetworkReceiveCb)(int error_code, uint64_t ipv4_address, uint8_t const *mac,
  uint8_t const *buffer, size_t size);
void cepton_sdk_listen_network_packet(FpCeptonNetworkReceiveCb cb);

#ifdef __cplusplus
} // extern "C" 
#endif
