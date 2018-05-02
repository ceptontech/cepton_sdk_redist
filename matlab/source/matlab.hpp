#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "cepton_sdk.h"
#include "cepton_sdk_matlab.h"

namespace cepton_sdk_matlab {

struct Error {
  CeptonSensorHandle handle;
  CeptonSensorErrorCode error_code;
  std::string msg;
  std::shared_ptr<void> data;
  std::size_t data_size;
};

class ErrorsListener {
 public:
  bool is_empty();
  std::shared_ptr<Error> queue_error();
  void clear_queued_error() { m_queued_error.reset(); }
  std::shared_ptr<Error> get_queued_error() { return m_queued_error; }

  void on_error(CeptonSensorHandle handle, CeptonSensorErrorCode error_code,
                const char *const error_msg, const void *const error_data,
                size_t error_data_size);

 private:
  std::mutex m_mutex;
  std::deque<std::shared_ptr<Error>> m_errors;
  std::shared_ptr<Error> m_queued_error;
};

extern ErrorsListener errors_listener;

struct Frame {
  CeptonSensorHandle handle;
  std::vector<CeptonSensorImagePoint> image_points;
};

class FramesListener {
 public:
  bool is_empty();
  std::shared_ptr<Frame> queue_frame();
  void clear_queued_frame() { m_queued_frame.reset(); }
  std::shared_ptr<Frame> get_queued_frame() { return m_queued_frame; }

  void on_image_points(CeptonSensorHandle handle, size_t n_points,
                       const CeptonSensorImagePoint *p_image_points);

 private:
  std::mutex m_mutex;
  std::deque<std::shared_ptr<Frame>> m_frames;
  std::shared_ptr<Frame> m_queued_frame;
};

extern FramesListener frames_listener;

}  // namespace cepton_sdk_matlab
