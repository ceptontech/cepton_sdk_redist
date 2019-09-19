/**
 * Sample code for custom packet replaying.
 */
#include <cepton_sdk/capture.hpp>
#include <cepton_sdk_api.hpp>

class CaptureReplay {
 public:
  CaptureReplay(const std::string& path) {
    CEPTON_CHECK_ERROR(m_capture.open_for_read(path));

    CEPTON_CHECK_ERROR(
        cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                                     CEPTON_SDK_CONTROL_DISABLE_NETWORK));
    CEPTON_CHECK_ERROR(cepton_sdk_clear());
  }

  ~CaptureReplay() {
    m_capture.close();
    if (cepton_sdk_is_initialized()) {
      CEPTON_CHECK_ERROR(cepton_sdk_clear());
    }
  }

  void run() {
    while (true) {
      cepton_sdk::Capture::PacketHeader header;
      const uint8_t* data;
      CEPTON_CHECK_ERROR(m_capture.next_packet(header, data));

      const cepton_sdk::SensorHandle handle =
          (cepton_sdk::SensorHandle)header.ip_v4 |
          CEPTON_SENSOR_HANDLE_FLAG_MOCK;
      CEPTON_CHECK_ERROR(cepton_sdk_mock_network_receive(
          handle, header.timestamp, data, header.data_size));
    }
  }

 private:
  cepton_sdk::Capture m_capture;
};

int main(int argc, char** argv) {
  if (argc < 2) return -1;
  const std::string capture_path = argv[1];

  // Initialize
  auto options = cepton_sdk::create_options();
  options.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  CEPTON_CHECK_ERROR(cepton_sdk::api::initialize(options));

  // Listen for points
  cepton_sdk::api::SensorImageFrameCallback callback;
  CEPTON_CHECK_ERROR(callback.initialize());
  CEPTON_CHECK_ERROR(
      callback.listen([](cepton_sdk::SensorHandle handle, std::size_t n_points,
                         const cepton_sdk::SensorImagePoint* c_image_points) {
        std::printf("Received %i points from sensor %lli\n", (int)n_points,
                    (long long)handle);
      }));

  // Run
  CaptureReplay replay(capture_path);
  replay.run();

  // Deinitialize
  cepton_sdk::deinitialize().ignore();
}