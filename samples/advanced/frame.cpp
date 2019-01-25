/**
 * Sample code for custom frame accumulation.
 */
#include <cepton_sdk_api.hpp>

int main(int argc, char **argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());

  // Get sensor
  while (cepton_sdk::get_n_sensors() == 0)
    cepton_sdk::api::check_error(cepton_sdk::api::wait(0.1f));
  cepton_sdk::SensorInformation sensor_info;
  cepton_sdk::api::check_error(
      cepton_sdk::get_sensor_information_by_index(0, sensor_info));

  // Create accumulator
  auto frame_options = cepton_sdk::create_frame_options();
  frame_options.mode = CEPTON_SDK_FRAME_TIMED;
  frame_options.length = 0.1f;
  cepton_sdk::util::FrameAccumulator accumulator(sensor_info);
  cepton_sdk::api::check_error(accumulator.set_options(frame_options));
  callback.listen(
      [&](cepton_sdk::SensorHandle handle, std::size_t n_points,
          const cepton_sdk::SensorImagePoint *const c_image_points) {
        if (handle != sensor_info.handle) return;
        accumulator.add_points(n_points, c_image_points);
      });

  // Listen
  accumulator.callback.listen(
      [&](int n_points,
          const cepton_sdk::SensorImagePoint *const c_image_points) {
        std::printf("Received %i points\n", n_points);
      });

  cepton_sdk::api::check_error(cepton_sdk::api::wait(5.0f));
}