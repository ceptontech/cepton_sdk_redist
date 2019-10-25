/**
 * Sample code for processing offline data from single sensor.
 */
#include <cepton_sdk_api.hpp>

#include "common.hpp"

struct Frame {
  int64_t timestamp;
  cepton_sdk::SensorHandle handle;
  std::vector<cepton_sdk::SensorImagePoint> image_points;
};

int main(int argc, char **argv) {
  check_help(argc, argv, "cepton_sdk_sample_process_single capture_path");
  if (!CEPTON_ASSERT(argc >= 2, "Capture path not provided!")) std::exit(1);
  const std::string capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());

  // Listen
  cepton_sdk::util::SingleConsumerQueue<Frame> queue;
  CEPTON_CHECK_ERROR(
      callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                          const cepton_sdk::SensorImagePoint *c_image_points) {
        // Add frame to queue
        auto frame = std::make_shared<Frame>();
        frame->timestamp = cepton_sdk::api::get_time();
        frame->handle = handle;
        frame->image_points.insert(frame->image_points.end(), c_image_points,
                                   c_image_points + n_points);
        queue.push(frame);
      }));

  while (!cepton_sdk::capture_replay::is_end()) {
    // Get frame
    if (queue.empty())
      CEPTON_CHECK_ERROR(cepton_sdk::capture_replay::resume_blocking(0.1f));
    const auto frame = queue.pop();
    if (!frame) continue;

    // Do processing...
  }

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
