/**
 * Sample code for general sdk usage.
 */
#include <vector>

#include <cepton_sdk_api.hpp>

#include "common.hpp"

class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Get sensor info
    cepton_sdk::SensorInformation sensor_info;
    CEPTON_CHECK_ERROR(cepton_sdk::get_sensor_information(handle, sensor_info));

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
  check_help(argc, argv, "cepton_sdk_sample_basic [capture_path]");
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  std::printf("Press Ctrl+C to stop\n");

  // Initialize
  auto options = cepton_sdk::create_options();
  options.frame.mode = CEPTON_SDK_FRAME_TIMED;
  options.frame.length = 0.1f;
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options, capture_path));

  // Get sensor
  std::printf("Waiting for sensor to connect...\n");
  while (cepton_sdk::get_n_sensors() == 0)
    CEPTON_CHECK_ERROR(cepton_sdk::api::wait(0.1f));
  cepton_sdk::SensorInformation sensor_info;
  CEPTON_CHECK_ERROR(
      cepton_sdk::get_sensor_information_by_index(0, sensor_info));
  std::printf("Sensor: %i\n", (int)sensor_info.serial_number);

  // Listen for frames
  std::printf("Listening for frames...\n");
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  FramesListener frames_listener;
  CEPTON_CHECK_ERROR(
      callback.listen(&frames_listener, &FramesListener::on_image_frame));

  // Run
  CEPTON_CHECK_ERROR(cepton_sdk::api::wait());

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
