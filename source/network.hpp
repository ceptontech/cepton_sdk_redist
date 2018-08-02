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

/// Object pool for storing large reusable temporary objects.
template <typename T>
class LargeObjectPool
    : public std::enable_shared_from_this<LargeObjectPool<T>> {
 public:
  std::shared_ptr<T> get() {
    T *ptr;
    if (m_free.empty()) {
      m_objects.emplace_back();
      ptr = &m_objects.back();
    } else {
      ptr = m_free.back();
      m_free.pop_back();
    }
    const auto this_ptr = this->shared_from_this();
    return std::shared_ptr<T>(
        ptr, [this_ptr, ptr](T *) { this_ptr->m_free.push_back(ptr); });
  }

 private:
  std::list<T> m_objects;
  std::vector<T *> m_free;
};

/// Single consumer concurrent queue
template <typename T>
class SimpleConcurrentQueue {
 public:
  int size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_queue.size();
  }

  bool empty() const { return size() == 0; }

  void clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    while (!m_queue.empty()) m_queue.pop();
  }

  void push(const std::shared_ptr<T> &value) {
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_queue.push(value);
    }
    m_condition_variable.notify_one();
  }

  std::shared_ptr<T> pop(int64_t timeout = 0) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_queue.empty()) {
      if (timeout == 0) return std::shared_ptr<T>();
      m_condition_variable.wait_for(
          lock, std::chrono::microseconds(timeout),
          [this]() -> bool { return !m_queue.empty(); });
    }
    if (m_queue.empty()) return std::shared_ptr<T>();
    const std::shared_ptr<T> value = std::move(m_queue.front());
    m_queue.pop();
    return value;
  }

 private:
  std::queue<std::shared_ptr<T>> m_queue;
  mutable std::mutex m_mutex;
  std::condition_variable m_condition_variable;
};

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
  void initialize();
  void deinitialize();

  uint16_t get_port() const;
  SensorError set_port(uint16_t port);

 private:
  uint16_t m_port = 8808;

  bool m_is_initialized = false;
  SimpleConcurrentQueue<Packet> m_packets;
  std::unique_ptr<SocketListener> m_listener;

  std::atomic<bool> m_is_running{false};
  std::unique_ptr<std::thread> m_listener_thread;
  std::unique_ptr<std::thread> m_worker_thread;
};

extern NetworkManager network_manager;

}  // namespace cepton_sdk
