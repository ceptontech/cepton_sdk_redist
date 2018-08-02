#include "cepton_sdk/capture_replay.hpp"

#include <chrono>

#include "cepton_sdk.hpp"
#include "cepton_sdk/core.hpp"
#include "cepton_sdk_util.hpp"

namespace cepton_sdk {

std::string CaptureReplay::filename() const {
  std::lock_guard<std::mutex> lock(m_capture_mutex);
  return m_capture.filename();
}

bool CaptureReplay::is_open() const {
  std::lock_guard<std::mutex> lock(m_capture_mutex);
  return m_capture.is_open();
}

SensorError CaptureReplay::open(std::string const &fname) {
  close();
  {
    std::lock_guard<std::mutex> lock(m_capture_mutex);
    if (!m_capture.open_for_read(fname)) return m_capture.get_error();
  }
  cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                               CEPTON_SDK_CONTROL_DISABLE_NETWORK);
  seek(0);
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::close() {
  util::ErrorAccumulator error = pause();
  m_capture.close();
  error = m_capture.get_error();
  m_is_end = true;
  if (sdk_manager.is_initialized()) {
    error = cepton_sdk_clear();
  }
  return error;
}

int64_t CaptureReplay::get_start_time() const {
  if (!is_open()) return 0;

  std::lock_guard<std::mutex> lock(m_capture_mutex);
  return m_capture.start_time_usec();
}

float CaptureReplay::get_position() const {
  if (!is_open()) return 0.0f;

  std::lock_guard<std::mutex> lock(m_capture_mutex);
  return 1e-6f * (float)m_capture.current_offset_usec();
}

float CaptureReplay::get_length() const {
  if (!is_open()) return 0.0f;

  std::lock_guard<std::mutex> lock(m_capture_mutex);
  return 1e-6f * (float)m_capture.total_offset_usec();
}

SensorError CaptureReplay::seek(float sec) {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;

  return run_paused([&]() { return seek_impl(int64_t(1e6f * sec)); });
}

SensorError CaptureReplay::seek_impl(int64_t usec) {
  {
    std::lock_guard<std::mutex> lock(m_capture_mutex);
    if (!m_capture.seek(usec)) return m_capture.get_error();
  }
  m_is_end = false;
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::run_paused(
    const std::function<SensorError()> &func) {
  const bool is_running_tmp = m_is_running;

  util::ErrorAccumulator error = pause();
  error = func();
  if (is_running_tmp) error = resume();
  return error;
}

SensorError CaptureReplay::set_enable_loop(bool enable_loop) {
  return run_paused([&]() {
    m_enable_loop = enable_loop;
    return CEPTON_SUCCESS;
  });
}

SensorError CaptureReplay::set_speed(float speed) {
  if ((speed < 1e-6f) || speed > 5.0f)
    return SensorError(CEPTON_ERROR_INVALID_ARGUMENTS, "Invalid replay speed!");

  return run_paused([&]() {
    m_speed = speed;
    return CEPTON_SUCCESS;
  });
}

SensorError CaptureReplay::resume_blocking_once() {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;
  if (m_is_running)
    return SensorError(CEPTON_ERROR_GENERIC, "Replay already running!");
  if (m_is_end) {
    if (m_enable_loop) {
      seek_impl(0);
      m_is_end = false;
    } else {
      return CEPTON_ERROR_EOF;
    }
  }

  feed_pcap_once(false);
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::resume_blocking(float sec) {
  if (sec < 0) return CEPTON_ERROR_INVALID_ARGUMENTS;
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;
  if (m_is_running)
    return SensorError(CEPTON_ERROR_GENERIC, "Replay already running!");
  if (m_is_end) {
    if (m_enable_loop) {
      seek_impl(0);
      m_is_end = false;
    } else {
      return CEPTON_ERROR_EOF;
    }
  }

  float start_sec = get_position();
  while (true) {
    if (m_is_end || ((get_position() - start_sec) > sec)) {
      break;
    }
    feed_pcap_once(false);
  }
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::resume() {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;

  pause();
  reset_time();
  m_is_running = true;
  m_feeder_thread.reset(new std::thread([this]() { feed_pcap(); }));
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::pause() {
  if (!is_open()) return CEPTON_SUCCESS;

  m_is_running = false;
  if (m_feeder_thread) {
    m_feeder_thread->join();
    m_feeder_thread.reset();
  }
  return CEPTON_SUCCESS;
}

void CaptureReplay::feed_pcap_once(bool enable_sleep) {
  if (m_is_end) return;

  int64_t timestamp;
  const Capture::PacketHeader *pkt_hdr;
  const uint8_t *pkt_data;
  int len;
  {
    std::lock_guard<std::mutex> lock(m_capture_mutex);
    timestamp = m_capture.start_time_usec() + m_capture.current_offset_usec();
    len = m_capture.next_packet(&pkt_hdr, &pkt_data);
  }
  if (len <= 0) {
    m_is_end = true;
    return;
  }

  if (enable_sleep) sleep_once();

  const CeptonSensorHandle handle =
      (CeptonSensorHandle)pkt_hdr->ip_v4 | CEPTON_SENSOR_HANDLE_FLAG_MOCK;
  cepton_sdk_mock_network_receive(handle, timestamp, pkt_data,
                                  pkt_hdr->data_size);
}

void CaptureReplay::reset_time() {
  m_start_ts_usec = util::get_timestamp_usec();
  {
    std::lock_guard<std::mutex> lock(m_capture_mutex);
    m_start_offset_usec = m_capture.current_offset_usec();
  }
}

void CaptureReplay::sleep_once() {
  const int64_t ts_usec = util::get_timestamp_usec() - m_start_ts_usec;
  int64_t offset_usec;
  {
    std::lock_guard<std::mutex> lock(m_capture_mutex);
    offset_usec = m_capture.current_offset_usec() - m_start_offset_usec;
  }
  offset_usec = int64_t((float)offset_usec / m_speed);

  const int64_t t_delta = offset_usec - ts_usec;
  if (std::abs(t_delta) > (int)1e6) {
    reset_time();
  } else if (t_delta <= 0) {
    // Catchup, do nothing here
  } else {
    std::this_thread::sleep_for(std::chrono::microseconds(t_delta));
  }
}

void CaptureReplay::feed_pcap() {
  while (m_is_running) {
    if (m_is_end) {
      if (m_enable_loop) {
        seek_impl(0);
        reset_time();
        m_is_end = false;
      } else {
        break;
      }
    }

    feed_pcap_once(true);
  }
  m_is_running = false;
}
}  // namespace cepton_sdk
