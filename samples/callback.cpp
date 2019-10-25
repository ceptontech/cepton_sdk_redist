/**
 * Sample code for callback usage.
 */
#include <cepton_sdk_api.hpp>

// Sample global callback.
void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                    const cepton_sdk::SensorImagePoint *c_image_points) {
  // Handle frame...
}

// Sample member callback.
class FramesListener {
 public:
  void on_image_frame(cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    // Handle frame...
  }
};

int main(int argc, char **argv) {
  // Initialize
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize());
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());

  // Listen lambda
  CEPTON_CHECK_ERROR(
      callback.listen([](cepton_sdk::SensorHandle handle, std::size_t n_points,
                         const cepton_sdk::SensorImagePoint *c_image_points) {
        //  Handle frame...
      }));

  // Listen global function
  CEPTON_CHECK_ERROR(callback.listen(on_image_frame));

  // Listen member function
  FramesListener frames_listener;
  CEPTON_CHECK_ERROR(
      callback.listen(&frames_listener, &FramesListener::on_image_frame));

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
