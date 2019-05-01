/**
 * Sample code for processing single sensor data.
 */
#include <memory>
#include <vector>

#include <cepton_sdk_api.hpp>

struct Frame {
  int64_t timestamp;
  cepton_sdk::SensorHandle handle;
  std::vector<cepton_sdk::SensorImagePoint> image_points;
};

int main(int argc, char **argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  auto options = cepton_sdk::create_options();
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());
  if (cepton_sdk::capture_replay::is_open())
    cepton_sdk::api::check_error(cepton_sdk::capture_replay::resume());

  cepton_sdk::util::SingleConsumerQueue<Frame> queue;
  callback.listen([&](cepton_sdk::SensorHandle handle, std::size_t n_points,
                      const cepton_sdk::SensorImagePoint *c_image_points) {
    auto frame = std::make_shared<Frame>();
    frame->timestamp = cepton_sdk::api::get_time();
    frame->handle = handle;
    frame->image_points.reserve(n_points);
    frame->image_points.insert(frame->image_points.end(), c_image_points,
                               c_image_points + n_points);
    queue.push(frame);
  });

  while (true) {
    const auto frame = queue.pop(0.01f);
    if (!frame) continue;
    // Do processing
  }
}
