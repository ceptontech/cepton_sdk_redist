#pragma once

#include <list>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

#include <asio.hpp>

#include "cepton_sdk.hpp"
#include "cepton_sdk_internal.h"
#include "cepton_sdk_util.hpp"

namespace cepton_sdk {

class SocketListener {
 public:
  SocketListener(uint16_t port);
  ~SocketListener();

  void run();
  void stop();

 private:
  void listen();

 public:
  // Arguments
  util::Callback<const asio::error_code &, CeptonSensorHandle, int,
                 const uint8_t *const>
      callback;

 private:
  mutable std::mutex m_mutex;
  asio::io_service m_io_service;
  asio::ip::udp::socket m_socket;
  asio::ip::udp::endpoint m_end_point;
  std::array<uint8_t, 65536> m_buffer;
};

class NetworkManager {
 private:
  struct Packet {
    CeptonSensorHandle handle;
    int64_t timestamp;
    std::vector<uint8_t> buffer;
  };

 public:
  static NetworkManager &instance() {
    static NetworkManager m_instance;
    return m_instance;
  }

  void initialize();
  void deinitialize();

  uint16_t get_port() const;
  SensorError set_port(uint16_t port);

 private:
  uint16_t m_port = 8808;

  bool m_is_initialized = false;
  util::SimpleConcurrentQueue<Packet> m_packets;
  std::unique_ptr<SocketListener> m_listener;

  std::atomic<bool> m_is_running{false};
  std::unique_ptr<std::thread> m_listener_thread;
  std::unique_ptr<std::thread> m_worker_thread;
};
}  // namespace cepton_sdk
