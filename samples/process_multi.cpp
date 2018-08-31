/**
 * Sample code for processing multiple sensor data.
 */
#include <iostream>
#include <string>
#include <vector>

#include <cepton_sdk_api.hpp>

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
    std::lock_guard<std::mutex> lock(m_mutex);

    auto& image_points = m_image_points_dict[handle];
    image_points.reserve(image_points.size() + n_points);
    image_points.insert(image_points.end(), c_image_points,
                        c_image_points + n_points);

    check_and_publish();
  }

 private:
  void check_and_publish() {
    const auto timestamp = cepton_sdk::api::get_time();

    if (timestamp < m_timestamp) return;
    if ((timestamp - m_timestamp) < int64_t(m_frame_length * 1e6f)) return;
    m_timestamp = timestamp;

    auto frame = std::make_shared<Frame>();
    frame->timestamp = timestamp;
    frame->image_points_dict = m_image_points_dict;
    m_image_points_dict.clear();
    queue.push(frame);
  }

 public:
  cepton_sdk::util::SimpleConcurrentQueue<Frame> queue;

 private:
  std::mutex m_mutex;
  float m_frame_length = 0.1f;
  int64_t m_timestamp = 0;
  std::map<cepton_sdk::SensorHandle, std::vector<cepton_sdk::SensorImagePoint>>
      m_image_points_dict;
};

int main(int argc, char** argv) {
  std::string capture_path;
  if (argc >= 2) capture_path = argv[1];

  auto options = cepton_sdk::create_options();
  cepton_sdk::api::check_error(
      cepton_sdk::api::initialize(options, capture_path));
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());
  if (cepton_sdk::capture_replay::is_open())
    cepton_sdk::api::check_error(cepton_sdk::capture_replay::resume());

  FrameAccumulator accumulator;
  callback.listen(&accumulator, &FrameAccumulator::on_image_frame);

  while (true) {
    const auto frame = accumulator.queue.pop(0.1f);
    if (!frame) continue;
    // Do processing
  }
}
