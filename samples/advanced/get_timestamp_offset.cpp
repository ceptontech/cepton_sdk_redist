/**
 * Sample code to get timestamp offset between sensor and host clock.
 * Useful for checking sensor timestamps or synchronizing to GPS.
 */
#include <atomic>
#include <iostream>
#include <string>
#include <vector>

#include <cepton_sdk_api.hpp>

int main(int argc, char** argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));

  // Wait for sensors
  while (cepton_sdk::get_n_sensors() == 0)
    cepton_sdk::api::check_error(cepton_sdk::api::wait(0.1f));

  // Listen
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());
  std::atomic<bool> is_done{false};
  callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint* c_image_points) {
    if (is_done) return;
    const auto& sensor_timestamp = c_image_points[n_points - 1].timestamp;
    const auto& host_timestamp = cepton_sdk::api::get_time();
    const int64_t timestamp_offset = sensor_timestamp - host_timestamp;
    std::printf("%lli\n", (long long int)timestamp_offset);
    is_done = true;
  });
  while (!is_done) cepton_sdk::api::check_error(cepton_sdk::api::wait(0.1f));
}
