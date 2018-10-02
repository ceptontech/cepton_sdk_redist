#include "matlab.hpp"

#include <cstring>

#include <iostream>

namespace cepton_sdk_matlab {

// -----------------------------------------------------------------------------
// ErrorsListener
// -----------------------------------------------------------------------------
ErrorsListener errors_listener;

void ErrorsListener::clear() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_errors.clear();
  m_queued_error.reset();
}

bool ErrorsListener::is_empty() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_errors.empty();
}

std::shared_ptr<Error> ErrorsListener::queue_error() {
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_errors.empty()) {
    m_queued_error.reset();
  } else {
    m_queued_error = m_errors.front();
    m_errors.pop_front();
  }
  return m_queued_error;
}

void ErrorsListener::on_error(CeptonSensorHandle handle,
                              CeptonSensorErrorCode error_code,
                              const char *const error_msg,
                              const void *const error_data,
                              size_t error_data_size) {
  auto error = std::make_shared<Error>();
  error->handle = handle;
  error->error_code = error_code;
  error->msg = error_msg;

  std::lock_guard<std::mutex> lock(m_mutex);

  m_errors.push_back(error);
  while (m_errors.size() > 100) {
    m_errors.pop_front();
  }
}

// -----------------------------------------------------------------------------
// FramesListener
// -----------------------------------------------------------------------------
FramesListener frames_listener;

void FramesListener::clear() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_frames.clear();
  m_queued_frame.reset();
}

bool FramesListener::is_empty() {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_frames.empty();
}

std::shared_ptr<Frame> FramesListener::queue_frame() {
  std::lock_guard<std::mutex> lock(m_mutex);

  if (m_frames.empty()) {
    m_queued_frame.reset();
  } else {
    m_queued_frame = m_frames.front();
    m_frames.pop_front();
  }
  return m_queued_frame;
}

void FramesListener::on_image_points(
    CeptonSensorHandle handle, size_t n_points,
    const CeptonSensorImagePoint *p_image_points) {
  auto frame = std::make_shared<Frame>();
  frame->handle = handle;
  frame->image_points.resize(n_points);
  memcpy(frame->image_points.data(), p_image_points,
         n_points * sizeof(CeptonSensorImagePoint));

  std::lock_guard<std::mutex> lock(m_mutex);

  m_frames.push_back(frame);
  while (m_frames.size() > 100) {
    m_frames.pop_front();
  }
}

}  // namespace cepton_sdk_matlab
