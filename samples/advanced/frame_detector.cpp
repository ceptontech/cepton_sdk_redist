/**
 * Sample code for custom frame detection.
 */
#include <cepton_sdk_api.hpp>

int main(int argc, char **argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());

  // Get sensor
  while (cepton_sdk::get_n_sensors() == 0)
    CEPTON_CHECK_ERROR(cepton_sdk::api::wait(0.1f));
  cepton_sdk::SensorInformation sensor_info;
  CEPTON_CHECK_ERROR(
      cepton_sdk::get_sensor_information_by_index(0, sensor_info));

  // Create detector
  cepton_sdk::util::FrameDetector<> detector(sensor_info);
  auto frame_options = cepton_sdk::create_frame_options();
  frame_options.mode = CEPTON_SDK_FRAME_COVER;
  CEPTON_CHECK_ERROR(detector.set_options(frame_options));
  const int stride = sensor_info.segment_count * sensor_info.return_count;
  CEPTON_CHECK_ERROR(callback.listen(
      [&](cepton_sdk::SensorHandle handle, std::size_t n_points,
          const cepton_sdk::SensorImagePoint *const c_image_points) {
        if (handle != sensor_info.handle) return;

        for (int i = 0; i < (int)n_points; i += stride) {
          auto &image_point = c_image_points[i];
          if (detector.update(image_point)) {
            auto &result = detector.previous_result();
            // `detector.period()` Frame period [seconds].
            // `result.timestamp` Frame timestamp [microseconds].

            // Handle frame...
          }
        }
      }));

  // Run
  CEPTON_CHECK_ERROR(cepton_sdk::api::wait(1.0f));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}