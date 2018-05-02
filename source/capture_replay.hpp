#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "cepton_sdk.h"
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

  virtual std::string filename() const;
  virtual bool is_open() const;
  virtual CeptonSensorErrorCode open(std::string const &fname);
  virtual CeptonSensorErrorCode close();

  virtual uint64_t get_start_time() const;
  virtual float get_position() const;
  virtual float get_length() const;
  virtual bool is_end() const { return m_is_end; }
  virtual CeptonSensorErrorCode rewind();
  virtual CeptonSensorErrorCode seek(float sec);

  virtual CeptonSensorErrorCode set_enable_loop(bool enable_loop);
  virtual bool get_enable_loop() const { return m_enable_loop; };
  virtual CeptonSensorErrorCode set_speed(float speed);
  virtual float get_speed() const { return m_speed; }

  virtual CeptonSensorErrorCode resume_blocking_once();
  virtual CeptonSensorErrorCode resume_blocking(float sec);

  virtual bool is_running() const { return m_is_running; }
  virtual CeptonSensorErrorCode resume();
  virtual CeptonSensorErrorCode pause();

 private:
  void reset_time();
  void sleep_once();
  void feed_pcap_once(bool enable_sleep);
  void feed_pcap();

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
