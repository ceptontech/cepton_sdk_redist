/**
 * Sample code for custom packet replaying.
 */
#include <iostream>

#include <cepton_sdk/capture.hpp>
#include <cepton_sdk_api.hpp>

class CaptureReplay {
 public:
  CaptureReplay(const std::string& path) {
    m_capture.open_for_read(path);

    cepton_sdk::api::check_error_code(m_capture.error_code());
    cepton_sdk::api::check_error_code(
        cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                                     CEPTON_SDK_CONTROL_DISABLE_NETWORK));

    seek(0);
  }

  ~CaptureReplay() {
    m_capture.close();
    cepton_sdk::set_mock_time_base(0);
  }

  void seek(int64_t usec) {
    m_capture.seek(usec);
    cepton_sdk::api::check_error_code(m_capture.error_code());

    cepton_sdk::api::check_error_code(cepton_sdk_clear_cache());
    cepton_sdk::api::check_error_code(
        cepton_sdk_set_mock_time_base(m_capture.start_time_usec() + usec));
  }

  void run() {
    while (true) {
      const cepton_sdk::Capture::PacketHeader* header;
      const uint8_t* data;
      int len = m_capture.next_packet(&header, &data);
      if (len <= 0) break;

      const uint64_t handle =
          (uint64_t)header->ip_v4 | CEPTON_SENSOR_HANDLE_FLAG_MOCK;
      cepton_sdk_mock_network_receive(handle, data, header->data_size);
    }
  }

 private:
  cepton_sdk::Capture m_capture;
};

int main(int argc, char** argv) {
  const std::string capture_path = argv[1];

  // Initialize sdk
  auto options = cepton_sdk::create_options();
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  cepton_sdk::api::check_error_code(cepton_sdk::api::initialize(options));

  // Listen for points
  cepton_sdk::api::SensorImageFramesCallback callback;
  cepton_sdk::api::check_error_code(callback.initialize());
  callback.listen([](cepton_sdk::SensorHandle handle, std::size_t n_points,
                     const cepton_sdk::SensorImagePoint* c_image_points) {
    std::printf("Received %lu points from %lu\n", n_points, handle);
  });

  CaptureReplay replay(capture_path);
  replay.seek(int(1.0 * 1e6));
  replay.run();
}