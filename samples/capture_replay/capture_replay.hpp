#pragma once

#include <atomic>
#include <thread>

#include "capture.hpp"

namespace cepton {

class CaptureReplay {
public:
  CaptureReplay():
    m_pcap_feeder(nullptr), paused(false) {}
  ~CaptureReplay() {
    close();
  }

  // Points to the pcap name, we will figure out the other files to load
  virtual bool load_capture(std::string const &fname);
  virtual bool is_open() const { return m_pcap.is_open(); }
  virtual void close() {
    if (m_pcap_feeder) {
      terminate_feeder_thread();
    }
    if (m_pcap.is_open())
      m_pcap.close();
  }

  virtual void skip_forward(float sec);
  virtual void skip_backward(float sec);
  virtual bool pause_resume() {
    paused = !paused;
    if (paused) {
      terminate_feeder_thread();
    }
    else {
      terminate_feeder.store(false);
      internal_resume();
    }
    return paused;
  };

  bool is_paused() const { return paused; }
protected:
  bool paused;
  
  // Current offset
  int64_t start_usec; // Real clock time of start

  virtual void before_packet() {}
  Capture m_pcap;
private:
  void terminate_feeder_thread() {
    terminate_feeder.store(true);
    if (m_pcap_feeder != nullptr) {
      m_pcap_feeder->join();  // Wait until it is done
      delete m_pcap_feeder;
      m_pcap_feeder = nullptr;
    }
  }

  // pcap based loading
  std::thread *m_pcap_feeder;
  std::atomic_bool terminate_feeder{ false };

  uint64_t serial_number;

  bool load_config(std::string const &fname);

  void internal_resume();

  void feed_pcap();
};


}
