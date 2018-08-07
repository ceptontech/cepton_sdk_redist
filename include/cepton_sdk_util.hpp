#pragma once

#include "cepton_sdk.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

#include <array>
#include <chrono>
#include <iostream>
#include <map>

namespace cepton_sdk {
namespace util {

#include "cepton_def.h"

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
template <typename T>
inline T square(T x) {
  return x * x;
}

/// Returns current unix timestamp [microseconds].
/**
 * This is the timestamp format used by all sdk functions.
 */
static int64_t get_timestamp_usec() {
  const auto t_epoch = std::chrono::steady_clock::now().time_since_epoch();
  // const auto t_epoch =
  // std::chrono::high_resolution_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(t_epoch).count();
}

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
/// Accumulates errors
/**
 * Useful for accumulating non fatal errors.
 * Currently, stores first error.
 */
class ErrorAccumulator {
 public:
  ErrorAccumulator() = default;
  ErrorAccumulator(const SensorError &error_) : error(error_) {}

  void operator=(const SensorError &new_error) {
    if (error) return;
    error = new_error;
  }

  operator bool() const { return error; }
  operator SensorError() const { return error; }
  operator SensorErrorCode() const { return error; }
  const SensorError &get() const { return error; }

 public:
  // Outputs
  SensorError error;
};

//------------------------------------------------------------------------------
// Points
//------------------------------------------------------------------------------
/// Convert image point to 3d point.
inline static void convert_image_point_to_point(float image_x, float image_z,
                                                float distance, float &x,
                                                float &y, float &z) {
  float hypotenuse_small = std::sqrt(square(image_x) + square(image_z) + 1.0f);
  float ratio = distance / hypotenuse_small;
  x = -image_x * ratio;
  y = ratio;
  z = -image_z * ratio;
}

/// 3d point class.
struct SensorPoint {
  int64_t timestamp;
  float x;
  float y;
  float z;
  float intensity;
  CeptonSensorReturnType return_type;

#ifdef SIMPLE
  uint8_t flags;
#else
  union {
    uint8_t flags;
    struct {
      uint8_t valid : 1;
      uint8_t saturated : 1;
    };
  };
  uint8_t reserved[2];
#endif
};

/// Convenience method to convert `CeptonSensorImagePoint` to
/// `cepton_sdk::SensorPoint`.
inline static void convert_sensor_image_point_to_point(
    const SensorImagePoint &image_point, SensorPoint &point) {
  point.timestamp = image_point.timestamp;
  point.intensity = image_point.intensity;
  point.return_type = image_point.return_type;
  point.flags = image_point.flags;

  convert_image_point_to_point(image_point.image_x, image_point.image_z,
                               image_point.distance, point.x, point.y, point.z);
}

// -----------------------------------------------------------------------------
// Transforms
// -----------------------------------------------------------------------------
/// 3d translation and rotation.
/**
 * For more functionality, use Eigen's Geometry module.
 */
class CompiledTransform {
 public:
  /// Create from translation and rotation.
  /**
   * @param translation Cartesian (x, y, z)
   * @param rotation Quaternion (x, y, z, w)
   */
  static CompiledTransform create(const float *const translation,
                                  const float *const rotation) {
    CompiledTransform compiled_transform;
    std::copy(translation, translation + 3,
              compiled_transform.translation.begin());

    // Convert quaternion to rotation matrix
    float x = rotation[0];
    float y = rotation[1];
    float z = rotation[2];
    float w = rotation[3];
    float xx = x * x;
    float xy = x * y;
    float xz = x * z;
    float xw = x * w;
    float yy = y * y;
    float yz = y * z;
    float yw = y * w;
    float zz = z * z;
    float zw = z * w;

    compiled_transform.rotation_m00 = 1 - 2 * (yy + zz);
    compiled_transform.rotation_m01 = 2 * (xy - zw);
    compiled_transform.rotation_m02 = 2 * (xz + yw);

    compiled_transform.rotation_m10 = 2 * (xy + zw);
    compiled_transform.rotation_m11 = 1 - 2 * (xx + zz);
    compiled_transform.rotation_m12 = 2 * (yz - xw);

    compiled_transform.rotation_m20 = 2 * (xz - yw);
    compiled_transform.rotation_m21 = 2 * (yz + xw);
    compiled_transform.rotation_m22 = 1 - 2 * (xx + yy);

    return compiled_transform;
  }

  /// Apply transformation to 3d position.
  void apply(float &x, float &y, float &z) {
    float x_tmp = x * rotation_m00 + y * rotation_m01 + z * rotation_m02;
    float y_tmp = x * rotation_m10 + y * rotation_m11 + z * rotation_m12;
    float z_tmp = x * rotation_m20 + y * rotation_m21 + z * rotation_m22;

    x_tmp += translation[0];
    y_tmp += translation[1];
    z_tmp += translation[2];

    x = x_tmp;
    y = y_tmp;
    z = z_tmp;
  }

 public:
  std::array<float, 3> translation;

  // Rotation matrix
  float rotation_m00 = 1.0f;
  float rotation_m01 = 0.0f;
  float rotation_m02 = 0.0f;
  float rotation_m10 = 0.0f;
  float rotation_m11 = 1.0f;
  float rotation_m12 = 0.0f;
  float rotation_m20 = 0.0f;
  float rotation_m21 = 0.0f;
  float rotation_m22 = 1.0f;
};

// -----------------------------------------------------------------------------
// Callbacks
// -----------------------------------------------------------------------------
template <typename TClass, typename... TArgs>
using MemberFunction = void (TClass::*)(TArgs...);

/// Expands SDK callback functionality.
/**
 * Allows for multiple callbacks to be registered.
 * Allows for registering lambdas and member functions.
 * See `samples/basic.cpp`.
 */
template <typename... TArgs>
class Callback {
 public:
  /// Clear all listeners.
  void clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_functions.clear();
  }

  /// Register std::function.
  SensorErrorCode listen(uint64_t id,
                         const std::function<void(TArgs...)> &func) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_functions.count(id)) return CEPTON_ERROR_TOO_MANY_CALLBACKS;
    m_functions[id] = func;
    return CEPTON_SUCCESS;
  }
  void unlisten(uint64_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_functions.erase(id);
  }

  /// Register global function.
  SensorErrorCode listen(void (*func)(TArgs...), uint64_t id = 0) {
    if (!id) id = (uint64_t)func;
    return listen(id, func);
  }
  void unlisten(void (*func)(TArgs...)) {
    const uint64_t id = (uint64_t)func;
    unlisten(id);
  }

  /// Register instance member function.
  template <typename TClass>
  SensorErrorCode listen(TClass *const instance,
                         MemberFunction<TClass, TArgs...> func,
                         uint64_t id = 0) {
    if (!id) id = (uint64_t)instance;
    return listen(
        id, [instance, func](TArgs... args) { (instance->*func)(args...); });
  }
  template <typename TClass>
  void unlisten(TClass *const instance, MemberFunction<TClass, TArgs...> func) {
    const uint64_t id = (uint64_t)instance;
    unlisten(id);
  }

  /// Emit callback.
  void emit(TArgs... args) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (const auto &iter : m_functions) {
      const auto &func = iter.second;
      func(args...);
    }
  }

  /// Emit callback.
  void operator()(TArgs... args) const { emit(args...); }

  /// Used for registering as c callback.
  static void global_on_callback(TArgs... args, void *const instance) {
    ((Callback *)instance)->emit(args...);
  }

 private:
  mutable std::mutex m_mutex;
  std::map<uint64_t, std::function<void(TArgs...)>> m_functions;
};

// -----------------------------------------------------------------------------
// Frame
// -----------------------------------------------------------------------------
namespace internal {
class MaxFilter {
 public:
  MaxFilter() { reset(); }

  void reset() {
    peak_found = false;
    peak_idx = -1;
    m_n = 0;
  }

  bool add_value(float v) {
    assert(!peak_found);
    if (peak_found) return true;

    ++m_n;
    if ((peak_idx < 0) || (v > peak_value)) {
      peak_idx = m_n - 1;
      peak_value = v;
    }
    if ((m_n - peak_idx) < min_n_after) return false;

    peak_found = true;
    return true;
  }

 public:
  // Arguments
  int min_n_after = 1;

  // Outputs
  bool peak_found = false;
  int peak_idx;
  float peak_value;

 private:
  int m_n;
};

class PeakFinder {
 public:
  PeakFinder() { reset(); }

  void reset() {
    peak_found = false;
    m_filter.reset();
  }

  bool add_value(float v) {
    assert(!peak_found);
    if (peak_found) return true;

    m_filter.min_n_after = min_n_after;
    m_filter.add_value((float)direction * v);
    if (!m_filter.peak_found) return false;

    peak_found = true;
    direction *= -1;
    peak_idx = m_filter.peak_idx;
    peak_value = -direction * m_filter.peak_value;
    return true;
  }

 public:
  // Arguments
  int min_n_after = 1;

  // Outputs
  bool peak_found;
  int direction = 1;
  int peak_idx;
  float peak_value;

 private:
  MaxFilter m_filter;
};

class CoverFrameDetector {
 public:
  CoverFrameDetector(const SensorInformation &sensor_info) {
    switch (sensor_info.model) {
      case HR80T:
      case HR80T_R2:
        is_model_supported = true;
        m_finder.min_n_after = 1200 / 4;
        break;
      case HR80W:
        is_model_supported = true;
        m_finder.min_n_after = 600 / 4;
        break;
      default:
        is_model_supported = false;
        break;
    }
  }

  void reset() {
    frame_found = false;
    m_finder.reset();
  }

  bool add_point(const SensorImagePoint &point) {
    assert(!frame_found);
    if (frame_found) return true;

    if (!m_finder.add_value(point.image_x)) return false;

    frame_found = true;
    direction = m_finder.direction;
    frame_idx = m_finder.peak_idx + 1;
    frame_x = m_finder.peak_value;
    return true;
  }

 public:
  // Outputs
  bool is_model_supported;
  bool frame_found;
  int direction;
  int frame_idx;
  float frame_x;

 private:
  PeakFinder m_finder;
};

class TimedFrameDetector {
 public:
  TimedFrameDetector(float length_) : length(length_) { reset(); }

  void reset() {
    frame_found = false;
    m_n = 0;
    m_t = 0;
  }

  bool add_point(const SensorImagePoint &point) {
    assert(!frame_found);
    if (frame_found) return true;

    ++m_n;

    if (m_t == 0) m_t = point.timestamp;
    const int64_t t_diff = point.timestamp - m_t;
    if ((t_diff > int64_t(1e6f * -0.5f)) && (t_diff < int64_t(1e6f * length)))
      return false;

    frame_found = true;
    frame_idx = m_n;
    frame_x = point.image_x;
    return true;
  }

 public:
  // Arguments
  float length;

  // Outputs
  bool frame_found;
  int frame_idx;
  float frame_x;

 private:
  int m_n;
  int64_t m_t;
};
}  // namespace internal

/// Detects frames in streaming sensor data.
class FrameDetector {
 public:
  FrameDetector(const SensorInformation &sensor_info)
      : m_sensor_info(sensor_info),
        m_timed_detector(0),
        m_cover_detector(sensor_info) {
    reset();
  }

  const FrameOptions &get_options() const { return m_options; }

  SensorErrorCode set_options(const FrameOptions &options) {
    m_options = options;

    switch (m_options.mode) {
      case CEPTON_SDK_FRAME_COVER:
        switch (m_sensor_info.model) {
          case VISTA_860:
            m_options.mode = CEPTON_SDK_FRAME_TIMED;
            m_options.length = 0.05f;
            break;
          default:
            if (!m_cover_detector.is_model_supported) {
              m_options.mode = CEPTON_SDK_FRAME_TIMED;
              m_options.length = 0.05f;
            }
            break;
        }
        break;
      case CEPTON_SDK_FRAME_CYCLE:
        switch (m_sensor_info.model) {
          case VISTA_860:
            m_options.mode = CEPTON_SDK_FRAME_TIMED;
            m_options.length = 0.1f;
            break;
          default:
            if (!m_cover_detector.is_model_supported) {
              m_options.mode = CEPTON_SDK_FRAME_TIMED;
              m_options.length = 0.1f;
            }
            break;
        }
        break;
    }

    switch (m_options.mode) {
      case CEPTON_SDK_FRAME_TIMED:
        if (!m_options.length) return CEPTON_ERROR_INVALID_ARGUMENTS;
        m_timed_detector.length = m_options.length;
        break;
    }

    reset();
    return CEPTON_SUCCESS;
  }

  /// Completely resets detector.
  /**
   * Only use if also clearing points accumulator.
   */
  void reset() {
    frame_found = false;
    m_n = 0;
    m_idx_0 = 0;
    m_timed_detector.reset();
    m_cover_detector.reset();
  }

  /// Returns true if frame found.
  /**
   * Automatically resets after frame is found.
   */
  bool add_point(const SensorImagePoint &point) {
    frame_found = false;
    frame_idx = -1;

    ++m_n;
    switch (m_options.mode) {
      case CEPTON_SDK_FRAME_TIMED:
        if (!m_timed_detector.add_point(point)) return false;
        frame_idx = m_timed_detector.frame_idx;
        frame_x = m_timed_detector.frame_x;
        break;
      case CEPTON_SDK_FRAME_COVER:
      case CEPTON_SDK_FRAME_CYCLE:
        if (!m_cover_detector.add_point(point)) return false;
        if ((m_options.mode == CEPTON_SDK_FRAME_CYCLE) &&
            (m_cover_detector.direction < 0))
          break;
        frame_idx = m_cover_detector.frame_idx;
        frame_x = m_cover_detector.frame_x;
        break;
      default:
        assert(false);
        return false;
    }

    if (frame_idx < 0) {
      m_idx_0 += m_n;
    } else {
      frame_found = true;
      const int idx_0 = m_n - frame_idx;
      frame_idx += m_idx_0;
      m_idx_0 = idx_0;
    }
    m_n = 0;
    m_timed_detector.reset();
    m_cover_detector.reset();
    return frame_found;
  }

 public:
  bool frame_found;
  int frame_idx;  /// Number of points in current frame.
  float frame_x;  /// Sanity check.

 private:
  SensorInformation m_sensor_info;
  FrameOptions m_options;

  int m_n;
  int m_idx_0;
  internal::TimedFrameDetector m_timed_detector;
  internal::CoverFrameDetector m_cover_detector;
};

/// Accumulates image points, and emits frames to callback.
/**
 * See `samples/frame.cpp`.
 */
class FrameAccumulator {
 public:
  FrameAccumulator(const SensorInformation &sensor_info)
      : m_sensor_info(sensor_info), m_frame_detector(sensor_info) {
    m_stride = (int)sensor_info.return_count * (int)sensor_info.segment_count;
    clear_impl();
  }

  FrameOptions get_options() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_frame_detector.get_options();
  }

  SensorErrorCode set_options(const FrameOptions &options) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto error_code = m_frame_detector.set_options(options);
    clear_impl();
    return error_code;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    clear_impl();
  }

  void add_points(int n_points, const SensorImagePoint *const c_image_points) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_frame_detector.get_options().mode == CEPTON_SDK_FRAME_STREAMING) {
      callback(n_points, c_image_points);
      return;
    }

    assert(n_points % m_stride == 0);
    const int i_0 = m_image_points.size();
    m_image_points.insert(m_image_points.end(), c_image_points,
                          c_image_points + n_points);
    for (int i = i_0; i < m_image_points.size(); i += m_stride) {
      const auto &image_point = m_image_points[i];
      if (!m_frame_detector.add_point(image_point)) continue;
      const int idx = m_frame_detector.frame_idx;
      assert(m_image_points[(idx - 1) * m_stride].image_x ==
             m_frame_detector.frame_x);

      if (m_i_frame > 0) callback(idx * m_stride, m_image_points.data());
      ++m_i_frame;

      m_image_points.erase(m_image_points.begin(),
                           m_image_points.begin() + idx * m_stride);
      i -= idx * m_stride;
    }
  }

 private:
  void clear_impl() {
    m_image_points.clear();
    m_i_frame = 0;
    m_frame_detector.reset();
  }

 public:
  // Arguments
  Callback<int, const SensorImagePoint *> callback;  /// Frames callback

 private:
  mutable std::mutex m_mutex;
  SensorInformation m_sensor_info;
  int m_stride;
  std::vector<SensorImagePoint> m_image_points;

  int m_i_frame;
  FrameDetector m_frame_detector;
};

#include "cepton_undef.h"

}  // namespace util
}  // namespace cepton_sdk
