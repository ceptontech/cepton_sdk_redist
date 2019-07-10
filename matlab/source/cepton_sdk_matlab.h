#ifndef CEPTON_SDK_MATLAB_H
#define CEPTON_SDK_MATLAB_H

#define CEPTON_SIMPLE
#include "cepton_sdk.h"
#undef CEPTON_SIMPLE

#ifdef __cplusplus
extern "C" {
#endif

#include "cepton_sdk_def.h"

/*
 * Duplicating some functions to provide interface for MATLAB.
 *
 * MATLAB doesn't support:
 * - Nested structs.
 * - Function pointers.
 * - Unions.
 */

CEPTON_EXPORT CeptonSensorErrorCode
cepton_sdk_matlab_initialize(int ver, CeptonSDKControl control_flags);
CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_deinitialize();
CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_clear_cache();

CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information(
    CeptonSensorHandle sensor_handle,
    struct CeptonSensorInformation *const sensor_info);
CEPTON_EXPORT CeptonSensorErrorCode
cepton_sdk_matlab_get_sensor_information_by_index(
    int sensor_index, struct CeptonSensorInformation *const sensor_info);

CEPTON_EXPORT int cepton_sdk_matlab_has_error();
CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_error(
    CeptonSensorHandle *const handle, CeptonSensorErrorCode *const error_code,
    size_t *const msg_size);

CEPTON_EXPORT int cepton_sdk_matlab_has_image_points();
CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_image_points(
    CeptonSensorHandle *const sensor_handle, size_t *const n_points);
CEPTON_EXPORT CeptonSensorErrorCode cepton_sdk_matlab_get_image_points_data(
    uint64_t *const timestamps, float *const image_x, float *const distances,
    float *const image_z, float *const intensities, uint8_t *const return_types,
    uint8_t *const flags);

#include "cepton_sdk_undef.h"

#ifdef __cplusplus
}  // extern "C"
#endif

#endif /* CEPTON_SDK_MATLAB_H */
