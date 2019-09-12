/**
 * Sample code for general sdk usage.
 */
#include <iostream>
#include <string>
#include <vector>

#undef CEPTON_ENABLE_EXCEPTIONS

#include <cepton_sdk_api.hpp>

class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Get sensor info
    cepton_sdk::SensorInformation sensor_info;
    cepton_sdk::api::check_error(
        cepton_sdk::get_sensor_information(handle, sensor_info));

    // Print info
    if (i_frame < 5) {
      std::printf("Received %i points from sensor %i\n", (int)n_points,
                  (int)sensor_info.serial_number);
    }
    ++i_frame;
  }

 private:
  std::size_t i_frame = 0;
};

int main(int argc, char **argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  options.frame.mode = CEPTON_SDK_FRAME_TIMED;
  options.frame.length = 0.1f;
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));

  // Get sensor
  std::printf("Waiting for sensor to connect...\n");
  while (cepton_sdk::get_n_sensors() == 0)
    cepton_sdk::api::check_error(cepton_sdk::api::wait(0.1f));
  cepton_sdk::SensorInformation sensor_info;
  cepton_sdk::api::check_error(
      cepton_sdk::get_sensor_information_by_index(0, sensor_info));
  std::printf("Sensor: %d\n", (int)sensor_info.serial_number);

  // Listen for frames
  std::printf("Listening for frames...\n");
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());
  FramesListener frames_listener;
  callback.listen(&frames_listener, &FramesListener::on_image_frame);

  // Run
  cepton_sdk::api::check_error(cepton_sdk::api::wait());

  // Deinitialize
  cepton_sdk::api::check_error(cepton_sdk::deinitialize());
}
