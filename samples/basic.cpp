#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <string>
#include <vector>

#include <cepton_sdk_api.hpp>

cepton_sdk::util::SensorImageFramesCallbackManager callback_manager;

void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                    const cepton_sdk::SensorImagePoint *c_image_points) {}

/// Frames callback.
class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Get sensor info
    cepton_sdk::SensorInformation sensor_info;
    cepton_sdk::api::check_error_code(
        cepton_sdk::get_sensor_information(handle, sensor_info));

    // Print info
    if (i_frame < 5) {
      std::printf("\n-- Frame --\n");
      std::printf("Sensor serial number: %d\n", (int)sensor_info.serial_number);
      std::printf("# Points: %d\n", (int)n_points);
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
  cepton_sdk::api::check_error_code(
      cepton_sdk::api::initialize(cepton_sdk::create_options(), capture_path));
  cepton_sdk::api::check_error_code(callback_manager.initialize());

  // Get sensor
  std::printf("Waiting for sensor to connect...\n");
  while (cepton_sdk::get_n_sensors() == 0)
    cepton_sdk::api::check_error_code(cepton_sdk::api::wait(0.1f));
  cepton_sdk::SensorInformation sensor_info;
  cepton_sdk::api::check_error_code(
      cepton_sdk::get_sensor_information_by_index(0, sensor_info));
  std::printf("Sensor connected: %d\n", (int)sensor_info.serial_number);

  // Listen for frames
  std::printf("Listening for frames...\n");

  // Listen lambda
  callback_manager.listen(
      [](cepton_sdk::SensorHandle handle, std::size_t n_points,
         const cepton_sdk::SensorImagePoint *c_image_points) {});

  // Listen global function
  callback_manager.listen(on_image_frame);

  // Listen member function
  FramesListener frames_listener;
  callback_manager.listen(&frames_listener, &FramesListener::on_image_frame);

  cepton_sdk::api::check_error_code(cepton_sdk::api::wait(5.0f));

  // Deinitialize (optional)
  cepton_sdk::api::check_error_code(cepton_sdk::deinitialize());
}
