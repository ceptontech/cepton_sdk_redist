#include "cepton_sdk/capture_replay.hpp"

#include <chrono>

#include "cepton_sdk/core.hpp"
#include "cepton_sdk_api.hpp"

namespace cepton_sdk {

// -----------------------------------------------------------------------------
// CaptureReplay
// -----------------------------------------------------------------------------
CaptureReplay::~CaptureReplay() { close(); }

std::string CaptureReplay::filename() const {
  util::LockGuard lock(m_capture_mutex);
  return m_capture.filename();
}

bool CaptureReplay::is_open() const {
  util::LockGuard lock(m_capture_mutex);
  return m_capture.is_open();
}

SensorError CaptureReplay::open(const std::string &filename) {
  auto error = open_impl(filename);
  if (error) close();
  return error;
}

SensorError CaptureReplay::open_impl(const std::string &filename) {
  close();

  {
    util::LockGuard lock(m_capture_mutex);
    const auto error = m_capture.open_for_read(filename);
    if (error) return error;
  }
  SensorError error;
  error = cepton_sdk_set_control_flags(CEPTON_SDK_CONTROL_DISABLE_NETWORK,
                                       CEPTON_SDK_CONTROL_DISABLE_NETWORK);
  if (error) return error;
  error = seek(0);
  if (error) return error;
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::close() {
  if (!is_open()) return CEPTON_SUCCESS;

  pause();
  m_is_end = true;
  {
    util::LockGuard lock(m_capture_mutex);
    m_capture.close();
  }
  cepton_sdk_clear();
  return CEPTON_SUCCESS;
}

int64_t CaptureReplay::get_start_time() const {
  if (!is_open()) return 0;

  util::LockGuard lock(m_capture_mutex);
  return m_capture.start_time();
}

float CaptureReplay::get_position() const {
  if (!is_open()) return 0.0f;

  util::LockGuard lock(m_capture_mutex);
  return 1e-6f * (float)m_capture.position();
}

float CaptureReplay::get_length() const {
  if (!is_open()) return 0.0f;

  util::LockGuard lock(m_capture_mutex);
  return 1e-6f * (float)m_capture.length();
}

SensorError CaptureReplay::seek(float position) {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;

  return run_paused([&]() { return seek_impl(int64_t(1e6f * position)); });
}

SensorError CaptureReplay::seek_impl(int64_t position) {
  {
    util::LockGuard lock(m_capture_mutex);
    const auto error = m_capture.seek(position);
    if (error) return error;
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
  if (speed != 0 && (speed < 1e-6f || speed > 5.0f))
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
      auto error = seek_impl(0);
      if (error) return error;
      m_is_end = false;
    } else {
      return CEPTON_ERROR_EOF;
    }
  }

  return feed_pcap_once(false);
}

SensorError CaptureReplay::resume_blocking(float duration) {
  if (duration < 0)
    return SensorError(CEPTON_ERROR_INVALID_ARGUMENTS, "Invalid duration!");
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;
  if (m_is_running)
    return SensorError(CEPTON_ERROR_GENERIC, "Replay already running!");
  if (m_is_end) {
    if (m_enable_loop) {
      auto error = seek_impl(0);
      if (error) return error;
      m_is_end = false;
    } else {
      return CEPTON_ERROR_EOF;
    }
  }

  const float start = get_position();
  while (true) {
    if (m_is_end || ((get_position() - start) > duration)) {
      break;
    }
    auto error = feed_pcap_once(false);
    if (error) return error;
  }
  return CEPTON_SUCCESS;
}

SensorError CaptureReplay::resume() {
  if (!is_open()) return CEPTON_ERROR_NOT_OPEN;

  auto error = pause();
  if (error) return error;
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

SensorError CaptureReplay::feed_pcap_once(bool enable_sleep) {
  if (m_is_end) return CEPTON_SUCCESS;

  Capture::PacketHeader header;
  const uint8_t *data;
  {
    util::LockGuard lock(m_capture_mutex);
    const auto error = m_capture.next_packet(header, data);
    if (error) {
      if (error.code() == CEPTON_ERROR_EOF) {
        m_is_end = true;
        return CEPTON_SUCCESS;
      }
      return error;
    }
  }

  if (enable_sleep && m_speed > 0) sleep_once();

  const CeptonSensorHandle handle =
      (CeptonSensorHandle)header.ip_v4 | CEPTON_SENSOR_HANDLE_FLAG_MOCK;
  CallbackManager::instance().network_cb.emit(handle, header.timestamp, data,
                                              header.data_size);
  return cepton_sdk_mock_network_receive(handle, header.timestamp, data,
                                         header.data_size);
}

void CaptureReplay::reset_time() {
  m_start_ts_usec = util::get_timestamp_usec();
  {
    util::LockGuard lock(m_capture_mutex);
    m_start_offset_usec = m_capture.position();
  }
}

void CaptureReplay::sleep_once() {
  const int64_t ts_usec = util::get_timestamp_usec() - m_start_ts_usec;
  int64_t offset_usec;
  {
    util::LockGuard lock(m_capture_mutex);
    offset_usec = m_capture.position() - m_start_offset_usec;
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

SensorError CaptureReplay::feed_pcap() {
  SensorError error;
  while (m_is_running) {
    if (m_is_end) {
      if (m_enable_loop) {
        error = seek_impl(0);
        if (error) break;
        reset_time();
        m_is_end = false;
      } else {
        break;
      }
    }

    error = feed_pcap_once(true);
    if (error) {
#ifdef CEPTON_INTERNAL
      api::log_error(error, "Capture replay failed!");
#endif
      break;
    }
  }
  m_is_running = false;
  return error;
}
}  // namespace cepton_sdk
