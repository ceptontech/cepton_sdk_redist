#include <cstdio>
#include <cstdlib>

#include <string>
#include <vector>

#include "cepton_sdk_api.hpp"

/// Frames callback.
class FramesListener {
 public:
  static void global_on_image_frame(
      cepton_sdk::SensorHandle handle, std::size_t n_points,
      const cepton_sdk::SensorImagePoint *c_image_points,
      void *const instance) {
    ((FramesListener *)instance)
        ->on_image_frame(handle, n_points, c_image_points);
  }

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
  FramesListener frames_listener;
  cepton_sdk::api::check_error_code(cepton_sdk::listen_image_frames(
      FramesListener::global_on_image_frame, &frames_listener));
  cepton_sdk::api::check_error_code(cepton_sdk::api::wait(1.0f));

  // Deinitialize (optional)
  cepton_sdk::api::check_error_code(cepton_sdk::deinitialize());
}
