/**
 * Sample code for sensors.
 */
#include <iostream>
#include <string>
#include <vector>

#include <cepton_sdk_api.hpp>

int main(int argc, char **argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::check_error(cepton_sdk::api::wait(5.0f));

  // Get all sensors
  const int n_sensors = cepton_sdk::get_n_sensors();
  for (int i = 0; i < n_sensors; ++i) {
    cepton_sdk::SensorInformation sensor_info;
    cepton_sdk::api::check_error(
        cepton_sdk::get_sensor_information_by_index(i, sensor_info));
    std::printf("%i: %s\n", (int)sensor_info.serial_number,
                sensor_info.model_name);
  }
}
