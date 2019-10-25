/**
 * Sample code for stray filter usage.
 */
#include <vector>

#include <cepton_sdk_api.hpp>

int main(int argc, char **argv) {
  // Initialize
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize());

  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  cepton_sdk::util::StrayFilter filter;
  CEPTON_CHECK_ERROR(
      callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                          const cepton_sdk::SensorImagePoint *c_image_points) {
        // Get sensor
        cepton_sdk::SensorInformation sensor_info;
        CEPTON_CHECK_ERROR(
            cepton_sdk::get_sensor_information(handle, sensor_info));

        // Copy points to buffer
        std::vector<cepton_sdk::SensorImagePoint> image_points;
        image_points.insert(image_points.begin(), c_image_points,
                            c_image_points + n_points);

        // Filter stray.
        filter.init(sensor_info);
        filter.run((int)n_points, image_points.data());
      }));

  // Run
  CEPTON_CHECK_ERROR(cepton_sdk::api::wait(5.0f));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
