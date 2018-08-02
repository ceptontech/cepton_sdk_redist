/**
 * Sample code for callback usage.
 */
#include <iostream>

#include <cepton_sdk_api.hpp>

// Sample global callback.
void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                    const cepton_sdk::SensorImagePoint *c_image_points) {}

// Sample member callback.
class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {}
};

int main(int argc, char **argv) {
  // Initialize
  cepton_sdk::api::check_error(cepton_sdk::api::initialize());
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());

  // Listen lambda
  callback.listen(0, [](cepton_sdk::SensorHandle handle, std::size_t n_points,
                     const cepton_sdk::SensorImagePoint *c_image_points) {});

  // Listen global function
  callback.listen(on_image_frame);

  // Listen member function
  FramesListener frames_listener;
  callback.listen(&frames_listener, &FramesListener::on_image_frame);
}
