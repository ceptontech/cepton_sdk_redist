/**
 * Sample code for custom frame accumulation.
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

  // Create accumulator
  auto frame_options = cepton_sdk::create_frame_options();
  frame_options.mode = CEPTON_SDK_FRAME_TIMED;
  frame_options.length = 0.1f;
  cepton_sdk::util::FrameAccumulator accumulator(sensor_info);
  CEPTON_CHECK_ERROR(accumulator.set_options(frame_options));
  CEPTON_CHECK_ERROR(callback.listen(
      [&](cepton_sdk::SensorHandle handle, std::size_t n_points,
          const cepton_sdk::SensorImagePoint *const c_image_points) {
        if (handle != sensor_info.handle) return;
        accumulator.add_points((int)n_points, c_image_points);
      }));

  // Listen
  CEPTON_CHECK_ERROR(accumulator.callback.listen(
      [&](int n_points,
          const cepton_sdk::SensorImagePoint *const c_image_points) {
        std::printf("Received %i points\n", n_points);
      }));

  CEPTON_CHECK_ERROR(cepton_sdk::api::wait(5.0f));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
