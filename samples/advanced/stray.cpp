/**
 * Sample code for stray filter usage. 
 */
#include <vector>

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
  cepton_sdk::util::StrayFilter filter;
  callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Get sensor
    cepton_sdk::SensorInformation sensor_info;
    cepton_sdk::api::check_error(
        cepton_sdk::get_sensor_information(handle, sensor_info));

    // Copy points to buffer
    std::vector<cepton_sdk::SensorImagePoint> image_points;
    image_points.insert(image_points.begin(), c_image_points,
                        c_image_points + n_points);

    // Filter stray
    filter.init(sensor_info);
    filter.run(n_points, image_points.data());
  });

  // Run
  cepton_sdk::api::check_error(cepton_sdk::api::wait(5.0f));
}
