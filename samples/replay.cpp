/**
 * Sample code for custom packet replaying.
 */
#include <iostream>

#include <cepton_sdk/capture.hpp>
#include <cepton_sdk_api.hpp>

class CaptureReplay {
 public:
  CaptureReplay(const std::string& path) {
    cepton_sdk::api::check_error(m_capture.open_for_read(path));

    cepton_sdk::api::check_error(
        cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                                     CEPTON_SDK_CONTROL_DISABLE_NETWORK));
    cepton_sdk::api::check_error(cepton_sdk_clear());
  }

  ~CaptureReplay() {
    m_capture.close();
    if (cepton_sdk_is_initialized()) {
      cepton_sdk::api::check_error(cepton_sdk_clear());
    }
  }

  void run() {
    while (true) {
      cepton_sdk::Capture::PacketHeader header;
      const uint8_t* data;
      cepton_sdk::api::check_error(m_capture.next_packet(header, data));

      const cepton_sdk::SensorHandle handle =
          (cepton_sdk::SensorHandle)header.ip_v4 |
          CEPTON_SENSOR_HANDLE_FLAG_MOCK;
      cepton_sdk::api::check_error(cepton_sdk_mock_network_receive(
          handle, header.timestamp, data, header.data_size));
    }
  }

 private:
  cepton_sdk::Capture m_capture;
};

int main(int argc, char** argv) {
  if (argc < 2) return -1;
  const std::string capture_path = argv[1];

  // Initialize sdk
  auto options = cepton_sdk::create_options();
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  cepton_sdk::api::check_error(cepton_sdk::api::initialize(options));

  // Listen for points
  cepton_sdk::api::SensorImageFrameCallback callback;
  cepton_sdk::api::check_error(callback.initialize());
  callback.listen([](cepton_sdk::SensorHandle handle, std::size_t n_points,
                     const cepton_sdk::SensorImagePoint* c_image_points) {
    std::printf("Received %i points from sensor %lli\n", (int)n_points,
                (long long)handle);
  });

  CaptureReplay replay(capture_path);
  replay.run();
}