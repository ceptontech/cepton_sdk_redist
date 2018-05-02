#pragma once

#include <memory>
#include <thread>
#include <vector>

#include <asio.hpp>

#include "cepton_sdk.h"

namespace cepton_sdk {

class NetworkSocket : public asio::ip::udp::socket {
 public:
  NetworkSocket(asio::io_service &io, uint16_t port);

 private:
  void listen();
  void handle_receive(const asio::error_code &error,
                      std::size_t bytes_transferred);

 private:
  uint8_t sensor_socket_buffer[4096];
  asio::ip::udp::endpoint end_point;
  int m_last_error;
};

class NetworkManager {
 public:
  void initialize();
  void deinitialize();

  uint16_t get_port() const;
  CeptonSensorErrorCode set_port(uint16_t port);

 private:
  mutable std::mutex m_mutex;

  uint16_t m_port = 8808;
  std::shared_ptr<asio::io_service> m_io_service_ptr;
  std::shared_ptr<NetworkSocket> m_sensor_socket_ptr;
  std::unique_ptr<std::thread> m_io_service_thread_ptr;
};

extern NetworkManager network_manager;

}  // namespace cepton_sdk
