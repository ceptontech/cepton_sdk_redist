/**
 * Sample code for general SDK usage.
 */
#include <cepton_sdk_api.hpp>

#include "common.hpp"

/// Sample points callback.
class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Get sensor info
    cepton_sdk::SensorInformation sensor_info;
    CEPTON_CHECK_ERROR(cepton_sdk::get_sensor_information(handle, sensor_info));

    // Convert points
    static thread_local std::vector<cepton_sdk::util::SensorPoint> points;
    points.resize(n_points);
    for (int i = 0; i < (int)n_points; ++i) {
      cepton_sdk::util::convert_sensor_image_point_to_point(c_image_points[i],
                                                            points[i]);
    }

    // Print
    std::printf("Received %i points from sensor %i\n", (int)n_points,
                (int)sensor_info.serial_number);
  }
};

int main(int argc, char **argv) {
  // Parse arguments
  check_help(argc, argv, "cepton_sdk_sample_basic [capture_path]");
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  std::printf("Press Ctrl+C to stop\n");

  // Initialize SDK
  auto options = cepton_sdk::create_options();

  // By default, return points every packet.

  // Uncomment to return points every frame.
  options.frame.mode = CEPTON_SDK_FRAME_COVER;

  // Uncomment to return points at fixed time interval.
  // options.frame.mode = CEPTON_SDK_FRAME_TIMED;
  // options.frame.length = 0.1f;

  // Wait short duration for sensors to connect.
  const bool enable_wait = true;

  std::printf("Initializing...\n");
  CEPTON_CHECK_ERROR(
      cepton_sdk::api::initialize(options, capture_path, enable_wait));

  // Get all sensors
  const int n_sensors = (int)cepton_sdk::get_n_sensors();
  for (int i = 0; i < n_sensors; ++i) {
    cepton_sdk::SensorInformation sensor_info;
    CEPTON_CHECK_ERROR(
        cepton_sdk::get_sensor_information_by_index(i, sensor_info));
    std::printf("Sensor: %i\n", (int)sensor_info.serial_number);
  }

  // Listen for points
  std::printf("Listening for points...\n");
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  FramesListener frames_listener;
  CEPTON_CHECK_ERROR(
      callback.listen(&frames_listener, &FramesListener::on_image_frame));

  // Run (sleep or run replay)
  CEPTON_CHECK_ERROR(cepton_sdk::api::wait(1.0));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
