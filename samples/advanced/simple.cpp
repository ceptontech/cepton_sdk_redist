#include <cepton_sdk_api.hpp>

int main(int argc, char **argv) {
  // Initialize SDK with default options
  CEPTON_CHECK_ERROR(
      cepton_sdk::api::initialize(cepton_sdk::create_options(), "", true));

  // Get all sensors
  for (int i = 0; i < cepton_sdk::get_n_sensors(); ++i) {
    cepton_sdk::SensorInformation sensor_info;
    CEPTON_CHECK_ERROR(
        cepton_sdk::get_sensor_information_by_index(i, sensor_info));
  }

  // Listen for points
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  CEPTON_CHECK_ERROR(
      callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                          const cepton_sdk::SensorImagePoint *c_image_points) {
        // Get sensor info
        cepton_sdk::SensorInformation sensor_info;
        CEPTON_CHECK_ERROR(
            cepton_sdk::get_sensor_information(handle, sensor_info));

        // Convert points
        std::vector<cepton_sdk::util::SensorPoint> points(n_points);
        for (int i = 0; i < n_points; ++i) {
          cepton_sdk::util::convert_sensor_image_point_to_point(
              c_image_points[i], points[i]);
        }
      }));

  // Sleep or do other work...

  // Deinitialize SDK
  cepton_sdk::deinitialize().ignore();
}