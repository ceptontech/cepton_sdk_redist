#ifndef CEPTON_SDK_MATLAB_H
#define CEPTON_SDK_MATLAB_H

#include "cepton_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "cepton_export.h"

struct EXPORT CeptonMatlabSensorInformation {
  CeptonSensorHandle handle;
  uint64_t serial_number;
  char model_name[28];
  uint32_t model;
  char firmware_version[32];

  float last_reported_temperature;  ///< [celsius]
  float last_reported_humidity;     ///< [%]
  float last_reported_age;          ///< [hours]
  float padding;

  uint64_t ptp_ts;

  uint8_t gps_ts_year;
  uint8_t gps_ts_month;
  uint8_t gps_ts_day;
  uint8_t gps_ts_hour;
  uint8_t gps_ts_min;
  uint8_t gps_ts_sec;

  uint8_t return_count;
  uint8_t reserved;

  uint32_t flags;
};

EXPORT CeptonSensorErrorCode
cepton_sdk_matlab_initialize(int ver, CeptonSDKControl control_flags);

EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information(
    CeptonSensorHandle sensor_handle,
    struct CeptonMatlabSensorInformation *const sensor_info);
EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information_by_index(
    int sensor_index, struct CeptonMatlabSensorInformation *const sensor_info);

EXPORT int cepton_sdk_matlab_has_error();
EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_error(
    CeptonSensorHandle *const handle, CeptonSensorErrorCode *const error_code,
    size_t *const msg_size);

EXPORT int cepton_sdk_matlab_has_image_points();
EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_image_points(
    CeptonSensorHandle *const sensor_handle, size_t *const n_points);
EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_image_points_data(
    uint64_t *const timestamps, float *const image_x, float *const distances,
    float *const image_z, float *const intensities,
    uint8_t *const return_numbers, uint8_t *const valid);

#ifdef __cplusplus
}  // extern "C"
#endif

#undef EXPORT

#endif /* CEPTON_SDK_MATLAB_H */
