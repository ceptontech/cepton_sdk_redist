/**
 * Sample code for sensor information.
 */
#include <string>

#include <cepton_sdk_api.hpp>

#include "common.hpp"

int main(int argc, char **argv) {
  check_help(argc, argv, "cepton_sdk_sample_sensor [capture_path]",
             "Print information for all sensors.");
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options, capture_path, true));

  // Get all sensors
  const int n_sensors = (int)cepton_sdk::get_n_sensors();
  for (int i = 0; i < n_sensors; ++i) {
    cepton_sdk::SensorInformation sensor_info;
    CEPTON_CHECK_ERROR(
        cepton_sdk::get_sensor_information_by_index(i, sensor_info));
    std::printf("%i: %s\n", (int)sensor_info.serial_number,
                sensor_info.model_name);
  }

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
