#ifndef CEPTON_SDK_MATLAB_H
#define CEPTON_SDK_MATLAB_H

#define SIMPLE
#include "cepton_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

#define COMPILING CEPTON_SDK_COMPILING
#include "cepton_def.h"

EXPORT CeptonSensorErrorCode
cepton_sdk_matlab_initialize(int ver, CeptonSDKControl control_flags);

EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information(
    CeptonSensorHandle sensor_handle,
    struct CeptonSensorInformation *const sensor_info);
EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information_by_index(
    int sensor_index, struct CeptonSensorInformation *const sensor_info);

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
    uint8_t *const return_types, uint8_t *const flags);

#include "cepton_undef.h"

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* CEPTON_SDK_MATLAB_H */
