#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

#include "cepton_sdk.hpp"
#include "cepton_sdk/capture.hpp"

namespace cepton_sdk {

/*
  PCAP file replay.

  Interface is not thread-safe.
*/
class CaptureReplay {
 public:
  CaptureReplay() {}
  ~CaptureReplay() { close(); }

  std::string filename() const;
  bool is_open() const;
  SensorError open(const std::string& filename);
  SensorError close();

  int64_t get_start_time() const;
  float get_position() const;
  float get_length() const;
  bool is_end() const { return m_is_end; }
  SensorError seek(float position);

  SensorError set_enable_loop(bool enable_loop);
  bool get_enable_loop() const { return m_enable_loop; };
  SensorError set_speed(float speed);
  float get_speed() const { return m_speed; }

  SensorError resume_blocking_once();
  SensorError resume_blocking(float duration);

  bool is_running() const { return m_is_running; }
  SensorError resume();
  SensorError pause();
  SensorError run_paused(const std::function<SensorError()>& func);

 private:
  SensorError open_impl(const std::string& filename);
  SensorError seek_impl(int64_t duration);
  void reset_time();
  void sleep_once();
  SensorError feed_pcap_once(bool enable_sleep);
  SensorError feed_pcap();

 private:
  bool m_enable_loop = false;
  float m_speed = 1.0f;

  std::unique_ptr<std::thread> m_feeder_thread;

  // Shared by replay thread
  std::atomic<bool> m_is_running{false};
  std::atomic<bool> m_is_end{true};
  mutable std::mutex m_capture_mutex;
  Capture m_capture;

  // Replay thread only
  int64_t m_start_offset_usec;  // Capture start time
  int64_t m_start_ts_usec;      // Clock start time
};

}  // namespace cepton_sdk
