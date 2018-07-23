/**
 * Sample code for custom networking.
 */
#include <iostream>

#include <asio.hpp>

#include <cepton_sdk_api.hpp>

using asio::ip::udp;

class SocketListener {
 public:
  SocketListener() : m_socket(m_io_service, udp::v4()) {
    m_socket.set_option(asio::socket_base::reuse_address(true));
    m_socket.bind(udp::endpoint(udp::v4(), 8808));
    listen();
  }

  ~SocketListener() {
    m_socket.close();
    m_io_service.stop();
  }

  void run() { m_io_service.run(); }

  void on_receive(const asio::error_code& error, std::size_t buffer_size) {
    const cepton_sdk::SensorHandle handle =
        m_end_point.address().to_v4().to_ulong();
    cepton_sdk::mock_network_receive(handle, m_buffer.data(), buffer_size);
  }

 private:
  void listen() {
    m_socket.async_receive_from(
        asio::buffer(m_buffer), m_end_point,
        [this](const asio::error_code& error, std::size_t buffer_size) {
          if (error) return;
          on_receive(error, buffer_size);
          listen();
        });
  }

 private:
  asio::io_service m_io_service;
  udp::socket m_socket;
  udp::endpoint m_end_point;
  std::array<uint8_t, 4096> m_buffer;
};

int main() {
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

  SocketListener manager;
  manager.run();
}