/**
 * Sample code for processing offline data from multiple sensors.
 */
#include <cepton_sdk_api.hpp>

#include "common.hpp"

struct Frame {
  int64_t timestamp;
  std::map<cepton_sdk::SensorHandle, std::vector<cepton_sdk::SensorImagePoint>>
      image_points_dict;
};

class FrameAccumulator {
 public:
  void on_image_frame(
      cepton_sdk::SensorHandle handle, std::size_t n_points,
      const cepton_sdk::SensorImagePoint* const c_image_points) {
    cepton_sdk::util::LockGuard lock(m_mutex);

    // Add points to buffer
    auto& image_points = m_image_points_dict[handle];
    image_points.insert(image_points.end(), c_image_points,
                        c_image_points + n_points);

    check_and_publish();
  }

 private:
  void check_and_publish() {
    // Check if frame done
    const auto timestamp = cepton_sdk::api::get_time();
    if ((timestamp - m_timestamp) < int64_t(m_frame_length * 1e6f)) return;
    m_timestamp = timestamp;

    // Add frame to queue
    auto frame = std::make_shared<Frame>();
    frame->timestamp = timestamp;
    frame->image_points_dict = m_image_points_dict;
    m_image_points_dict.clear();
    queue.push(frame);
  }

 public:
  cepton_sdk::util::SingleConsumerQueue<Frame> queue;

 private:
  std::timed_mutex m_mutex;
  float m_frame_length = 0.1f;
  int64_t m_timestamp = 0;
  std::map<cepton_sdk::SensorHandle, std::vector<cepton_sdk::SensorImagePoint>>
      m_image_points_dict;
};

int main(int argc, char** argv) {
  check_help(argc, argv, "cepton_sdk_sample_process_multi capture_path");
  if (!CEPTON_ASSERT(argc >= 2, "Capture path not provided!")) std::exit(1);
  const std::string capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());

  // Listen
  FrameAccumulator accumulator;
  CEPTON_CHECK_ERROR(
      callback.listen(&accumulator, &FrameAccumulator::on_image_frame));

  while (!cepton_sdk::capture_replay::is_end()) {
    // Get frame
    if (accumulator.queue.empty())
      CEPTON_CHECK_ERROR(cepton_sdk::capture_replay::resume_blocking(0.1f));
    const auto frame = accumulator.queue.pop();
    if (!frame) continue;

    // Do processing...
  }

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}
