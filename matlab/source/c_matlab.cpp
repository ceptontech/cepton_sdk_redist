#include "matlab.hpp"

static void global_on_error(CeptonSensorHandle handle,
                            CeptonSensorErrorCode error_code,
                            const char *const error_msg,
                            const void *const error_data,
                            size_t error_data_size, void *const user_data) {
  cepton_sdk_matlab::errors_listener.on_error(handle, error_code, error_msg,
                                              error_data, error_data_size);
}

static void global_on_image_points(CeptonSensorHandle handle, size_t n_points,
                                   const CeptonSensorImagePoint *p_image_points,
                                   void *const user_data) {
  cepton_sdk_matlab::frames_listener.on_image_points(handle, n_points,
                                                     p_image_points);
}

CeptonSensorErrorCode cepton_sdk_matlab_initialize(
    int ver, CeptonSDKControl control_flags) {
  auto options = cepton_sdk_create_options();
  options.control_flags = control_flags;
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  CeptonSensorErrorCode error_code =
      cepton_sdk_initialize(ver, &options, global_on_error, nullptr);
  if (error_code) return error_code;

  error_code = cepton_sdk_listen_image_frames(global_on_image_points, nullptr);
  if (error_code) return error_code;

  return CEPTON_SUCCESS;
}

CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information(
    CeptonSensorHandle handle, CeptonSensorInformation *const info) {
  CeptonSensorInformation info_tmp;
  CeptonSensorErrorCode error_code =
      cepton_sdk_get_sensor_information(handle, &info_tmp);
  if (error_code) return error_code;
  *info = *reinterpret_cast<const CeptonSensorInformation *>(&info_tmp);
  return CEPTON_SUCCESS;
}

CeptonSensorErrorCode cepton_sdk_matlab_get_sensor_information_by_index(
    int index, CeptonSensorInformation *const info) {
  CeptonSensorInformation info_tmp;
  CeptonSensorErrorCode error_code =
      cepton_sdk_get_sensor_information_by_index(index, &info_tmp);
  if (error_code) return error_code;
  *info = *reinterpret_cast<const CeptonSensorInformation *>(&info_tmp);
  return CEPTON_SUCCESS;
}

int cepton_sdk_matlab_has_error() {
  return (int)(!cepton_sdk_matlab::errors_listener.is_empty());
}

CeptonSensorErrorCode cepton_sdk_matlab_get_error(
    CeptonSensorHandle *const handle, int *const error_code,
    size_t *const msg_size) {
  auto error_data = cepton_sdk_matlab::errors_listener.queue_error();
  if (!error_data) return CEPTON_ERROR_GENERIC;

  *handle = error_data->handle;
  *error_code = error_data->error_code;
  *msg_size = error_data->msg.size();
  return CEPTON_SUCCESS;
}

int cepton_sdk_matlab_has_image_points() {
  return (int)(!cepton_sdk_matlab::frames_listener.is_empty());
}

CeptonSensorErrorCode cepton_sdk_matlab_get_image_points(
    CeptonSensorHandle *const handle, size_t *const n_points) {
  auto frame = cepton_sdk_matlab::frames_listener.queue_frame();
  if (!frame) return CEPTON_ERROR_GENERIC;

  *handle = frame->handle;
  *n_points = frame->image_points.size();
  return CEPTON_SUCCESS;
}

CeptonSensorErrorCode cepton_sdk_matlab_get_image_points_data(
    uint64_t *const timestamps, float *const image_x, float *const distances,
    float *const image_z, float *const intensities, uint8_t *const return_types,
    uint8_t *const flags) {
  auto frame = cepton_sdk_matlab::frames_listener.get_queued_frame();
  cepton_sdk_matlab::frames_listener.clear_queued_frame();
  if (!frame) return CEPTON_ERROR_GENERIC;

  auto &image_points = frame->image_points;
  for (std::size_t i = 0; i < image_points.size(); ++i) {
    auto &image_point = image_points[i];
    timestamps[i] = image_point.timestamp;
    image_x[i] = image_point.image_x;
    distances[i] = image_point.distance;
    image_z[i] = image_point.image_z;
    intensities[i] = image_point.intensity;
    return_types[i] = image_point.return_type;
    flags[i] = image_point.flags;
  }
  return CEPTON_SUCCESS;
}
