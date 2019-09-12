#pragma once

#include "cepton_sdk.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <queue>

namespace cepton_sdk {
namespace util {

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
inline int64_t to_usec(float sec) { return int64_t(sec * 1e6); }
inline float from_usec(int64_t usec) { return float(usec) * 1e-6f; }
const int64_t second_usec(to_usec(1.0f));
const int64_t hour_usec(to_usec(60.0f * 60.0f));
const int64_t hour_sec(3600);

static const float PI = 3.14159265359f;
inline float to_degrees(float val) { return val * (180.0f / PI); }
inline float to_radians(float val) { return val * (PI / 180.0f); }

template <typename T>
inline T square(T x) {
  return x * x;
}

/// Returns current unix timestamp [microseconds].
/**
 * This is the timestamp format used by all sdk functions.
 */
inline int64_t get_timestamp_usec() {
  // const auto t_epoch =
  // std::chrono::high_resolution_clock::now().time_since_epoch();
  const auto t_epoch = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(t_epoch).count();
}

// -----------------------------------------------------------------------------
// Concurrent
// -----------------------------------------------------------------------------
class LockGuard {
 public:
  explicit LockGuard(std::timed_mutex &mutex) : m_mutex(mutex) {
    m_is_locked = m_mutex.try_lock_for(std::chrono::seconds(1));
    if (m_is_locked) return;
    CEPTON_ASSERT(false, "Deadlock!");
  }

  ~LockGuard() {
    if (m_is_locked) m_mutex.unlock();
  }

  LockGuard(const LockGuard &) = delete;
  LockGuard &operator=(const LockGuard &) = delete;

 private:
  bool m_is_locked = false;
  std::timed_mutex &m_mutex;
};

/// Object pool for storing large reusable temporary objects.
template <typename T>
class LargeObjectPool
    : public std::enable_shared_from_this<LargeObjectPool<T>> {
 public:
  std::shared_ptr<T> get() {
    LockGuard lock(m_mutex);
    T *ptr;
    if (m_free.empty()) {
      m_objects.emplace_back();
      ptr = &m_objects.back();
    } else {
      ptr = m_free.back();
      m_free.pop_back();
    }

    auto this_ptr = this->shared_from_this();
    return std::shared_ptr<T>(ptr, [this, this_ptr](T *const ptr_tmp) {
      LockGuard lock_tmp(m_mutex);
      m_free.push_back(ptr_tmp);
    });
  }

 private:
  std::timed_mutex m_mutex;
  std::list<T> m_objects;
  std::vector<T *> m_free;
};

template <typename T>
class SingleConsumerQueue {
 public:
  int size() const { return m_size; }

  bool empty() const { return size() == 0; }

  void clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_queue = std::queue<std::shared_ptr<T>>();
    m_size = (int)m_queue.size();
  }

  int push(const std::shared_ptr<T> &value, int max_size = 0) {
    int n_dropped = 0;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_queue.push(value);
      if (max_size > 0) {
        while (m_queue.size() > max_size) {
          m_queue.pop();
          ++n_dropped;
        }
      }
      m_size = (int)m_queue.size();
    }
    m_condition_variable.notify_one();
    return n_dropped;
  }

  std::shared_ptr<T> pop(float timeout = 0.0f) {
    if (!timeout && empty()) return nullptr;
    std::unique_lock<std::mutex> lock(m_mutex);
    if (empty()) {
      m_condition_variable.wait_for(lock,
                                    std::chrono::microseconds(to_usec(timeout)),
                                    [this]() -> bool { return !empty(); });
    }
    if (empty()) return nullptr;
    const std::shared_ptr<T> value = m_queue.front();
    m_queue.pop();
    m_size = (int)m_queue.size();
    return value;
  }

 private:
  mutable std::mutex m_mutex;
  std::atomic<int> m_size{0};
  std::queue<std::shared_ptr<T>> m_queue;
  std::condition_variable m_condition_variable;
};

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
/// Accumulates errors
/**
 * DEPRICATED: use `cepton_sdk::SensorErrorWrapper`.
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
inline void convert_image_point_to_point(float image_x, float image_z,
                                         float distance, float &x, float &y,
                                         float &z) {
  float hypotenuse_small = std::sqrt(square(image_x) + square(image_z) + 1.0f);
  float ratio = distance / hypotenuse_small;
  x = -image_x * ratio;
  y = ratio;
  z = -image_z * ratio;
}
/// Convert image point to spherical coordinates. Convention used is the
/// "mathematics" convention. Refer to this link for a diagram
/// https://en.wikipedia.org/wiki/Spherical_coordinate_system#/media/File:3D_Spherical_2.svg
/// Positive azimuth is defined as clockwise rotation around the z-axis
/// with 0 azimuth pointing along the positive x-axis.
/// Inclination is defined as the angle from the positive z-axis. Both
/// inclination and azimuth will always fall in [0, pi)
inline void convert_image_point_to_polar(float image_x, float image_z,
                                         float &inclination, float &azimuth) {
  azimuth = std::atan2(image_x, 1.0f) + (PI / 2.f);
  inclination = (PI / 2.f) - std::atan2(-image_z, 1.0f);
}

/// 3d point class.
/**
 * Can't subclass from SensorImagePoint, needs to be POD.
 */
struct SensorPoint {
  int64_t timestamp;  ///< Unix time [microseconds].
  float image_x;      ///< x image coordinate.
  float distance;     ///< Distance [meters].
  float image_z;      ///< z image coordinate.
  float intensity;    ///< Diffuse reflectance.
  CeptonSensorReturnType return_type;

#ifdef CEPTON_SIMPLE
  /// bit flags
  /**
   * 1. `valid`: If `false`, then the distance and intensity are invalid.
   * 2. `saturated`: If `true`, then the intensity is invalid. Also, the
   * distance is valid, but inaccurate.
   *
   */
  uint8_t flags;
#else
  union {
    uint8_t flags;
    struct {
      uint8_t valid : 1;
      uint8_t saturated : 1;
    };
  };
#endif
  uint8_t reserved[5];

  float x;  ///< x cartesian coordinate
  float y;  ///< y cartesian coordinate
  float z;  ///< z cartesian coordinate
};

/**
 * @brief The OrganizedCloud struct
 * An organized version of the cepton point cloud.
 */
struct OrganizedCloud {
  /**
   * @brief timestamp_start The time of the oldest point in the cloud
   */
  int64_t timestamp_start;
  /**
   * @brief timestamp_end The time of the newest point in the cloud
   */
  int64_t timestamp_end;
  /**
   * @brief height Height of the cloud.  Represents how many rows there are in
   * the cloud
   */
  int height;
  /**
   * @brief width Width of the cloud.  Represents how many columns there are in
   * the cloud
   */
  int width;

  /**
   * @brief n_returns Number of return represented by the cloud.
   */
  int n_returns;

  /**
   * @brief The CellInfo struct
   *
   */
  struct CellInfo {
    /**
     * @brief occupied_cell Is the cell at this index occupied with a point.  If
     * false can't assume this represents free space.
     */
    bool occupied_cell = false;
    /**
     * @brief original_index Index of the point that was used to generate the
     * organized point. Can be used to match back with orginial data if
     * required. Should only be use if occupied_cell is true.
     */
    int original_index = -1;
  };

  /**
   * @brief getIndex Returns the index of the point corresponding to the inputed
   * row, col and return number.
   * @param[in] row Row index
   * @param[in] col Col index
   * @param[in] n_return Return index
   * @return
   */
  int getIndex(int row, int col, int n_return) {
    return (row * width + col) * n_returns + n_return;
  }

  /**
   * @brief info_cells Vector of cell info which provide information about the
   * matching points
   */
  std::vector<CellInfo> info_cells;

  /**
   * @brief points Vector of organized points.
   * Stored in Return, Row, Col order.
   * So to get a point at row 10, col 15, return 1 would be points[(row * width
   * + col) n_returns + return
   */
  std::vector<CeptonSensorImagePoint> points;
};

/**
 * @brief The Organizer class
 * Performs organization on cepton unorganized points.
 * Creates an angular grid, places each point within that grid and outputs
 * a point for each location in the grid in a row/col format.
 * Thread safe.
 * Defaults to a 0.4deg spaced grid
 */
class Organizer {
 public:
  /**
   * @brief Organizer
   * @param sensor_info Sensor info for organizer.  Used to set min/max angles
   */
  Organizer(cepton_sdk::SensorInformation sensor_info);

  /**
   * @brief organize_points
   * @param[in] num_points_in Number of unorganized points
   * @param[in] n_returns Number of returns
   * @param[in] unorganized_points Unorganized points to proces
   * @param[out] organized_points Points in organized form
   */
  void organize_points(const int num_points_in, const int n_returns,
                       const CeptonSensorImagePoint *const unorganized_points,
                       cepton_sdk::util::OrganizedCloud &organized_points);

  enum class OrganizerMode {
    RECENT,  ///< Output the most recent point from the frame that fell within
             ///< the grid
    CENTER   ///< Output the center of the grid.  Uses median point distance.
  };

  struct OrganizerSettings {
    float horizontal_range_radians = to_radians(70.f);
    float vertical_range_radians = to_radians(30.f);
    float horizontal_bin_size_radians = to_radians(0.4f);
    float vertical_bin_size_radians = to_radians(0.4f);
    OrganizerMode mode = OrganizerMode::RECENT;
  };

  /**
   * @brief mode
   * @param mode Change the mode of the organizer.
   * [RECENT] Points are the most recent which fill within the grid.
   * [CENTER] Points outputted are at the center of the grid.  More even spacing
   * but less accurate.
   */
  void mode(OrganizerMode mode);

  /**
   * @brief binSize Change the bin size of the organizer
   * @param bin_size The horizontal and vertical bin size to set. In radians
   */
  void binSize(float bin_size);

  /**
   * @brief settings
   * @param organizer_settings Change organizer settings
   */
  void settings(OrganizerSettings organizer_settings);

  /**
   * @brief settings
   * @return The settings the organizer is using
   */
  OrganizerSettings settings();

 private:
  struct GridIndex {
    int row = -1;  //-1 is invalid
    int col = -1;  //-1 is invalid
  };

  struct ImageXZ {
    float X;
    float Z;
  };

  /**
   * @brief getIndex
   * @param[in] cloud
   * @param[in] X
   * @param[in] Z
   * @return The index in the grid that Cepton sensor X,Z coordinates match with
   */
  GridIndex getGridIndex(const OrganizedCloud &cloud, float X, float Z);

  /**
   * @brief getXZ
   * @param[in] width
   * @param[in] height
   * @param[in] row
   * @param[in] col
   * @return The Cepton sensor coordinates of a row and column in the grid
   */
  ImageXZ getXZ(int width, int height, int row, int col);

  /**
   * @brief m_settings Organizer settings
   */
  OrganizerSettings m_settings;

  /**
   * @brief Vector of organized points with distance values of 0
   */
  std::vector<CeptonSensorImagePoint> empty_points;

  std::mutex m_organizer_mutex;
};

/// Convenience method to convert `cepton_sdk::SensorImagePoint` to
/// `cepton_sdk::SensorPoint`.
inline void convert_sensor_image_point_to_point(
    const SensorImagePoint &image_point, SensorPoint &point) {
  *(SensorImagePoint *)(&point) = image_point;

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
  std::array<float, 3> translation = {{0, 0, 0}};

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
  virtual ~Callback() = default;

  /// Clear all listeners.
  void clear() {
    LockGuard lock(m_mutex);
    m_i_callback = 0;
    m_functions.clear();
  }

  /// Register std::function.
  SensorError listen(const std::function<void(TArgs...)> &func,
                     uint64_t *const id = nullptr) {
    LockGuard lock(m_mutex);
    if (id) *id = m_i_callback;
    m_functions[m_i_callback] = func;
    ++m_i_callback;
    return CEPTON_SUCCESS;
  }
  SensorError unlisten(uint64_t id) {
    SensorErrorWrapper error("cepton_sdk::util::Callback::unlisten");
    LockGuard lock(m_mutex);
    if (!m_functions.count(id)) {
      error =
          SensorError(CEPTON_ERROR_INVALID_ARGUMENTS, "Invalid function id");
      return error;
    }
    m_functions.erase(id);
    return CEPTON_SUCCESS;
  }

  /// Register instance member function.
  template <typename TClass>
  SensorError listen(TClass *const instance,
                     MemberFunction<TClass, TArgs...> func,
                     uint64_t *const id = nullptr) {
    return listen(
        [instance, func](TArgs... args) { (instance->*func)(args...); }, id);
  }

  /// Emit callback.
  void operator()(TArgs... args) const {
    LockGuard lock(m_mutex);
    for (const auto &iter : m_functions) {
      const auto &func = iter.second;
      func(args...);
    }
  }

  /// Used for registering as c callback.
  static void global_on_callback(TArgs... args, void *const instance) {
    ((Callback *)instance)->operator()(args...);
  }

 private:
  mutable std::timed_mutex m_mutex;
  int m_i_callback = 0;
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
        m_finder.min_n_after = 1200 / 4;  // 1/4 frame
        break;
      case HR80W:
        is_model_supported = true;
        m_finder.min_n_after = 600 / 4;  // 1/4 frame
        break;
      case SORA_200:
        is_model_supported = true;
        m_finder.min_n_after = 10;
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
    if ((t_diff > int64_t(1e6 * -0.5)) && (t_diff < int64_t(1e6 * length)))
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

// Detects scanlines in streaming data.
class ScanlineDetector {
 public:
  ScanlineDetector(const SensorInformation &sensor_info) {
    is_model_supported = true;
    m_finder.min_n_after = 2;
    reset();
  }

  void reset() {
    scanline_found = false;
    m_n = 0;
    m_idx_0 = 0;
    m_finder.reset();
  }

  bool add_point(const SensorImagePoint &point) {
    scanline_found = false;
    scanline_idx = -1;

    ++m_n;
    if (!m_finder.add_value(point.image_z)) return false;

    scanline_found = true;
    direction = m_finder.direction;
    scanline_idx = m_finder.peak_idx + 1;
    scanline_z = m_finder.peak_value;

    const int idx_0 = m_n - scanline_idx;
    scanline_idx += m_idx_0;
    m_idx_0 = idx_0;

    m_n = 0;
    m_finder.reset();
    return true;
  }

 public:
  // Outputs
  bool is_model_supported;
  bool scanline_found;
  int direction;
  int scanline_idx;
  float scanline_z;

 private:
  int m_n;
  int m_idx_0;
  internal::PeakFinder m_finder;
};

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

  SensorError set_options(const FrameOptions &options) {
    m_options = options;

    // Fix options
    switch (m_options.mode) {
      case CEPTON_SDK_FRAME_COVER:
      case CEPTON_SDK_FRAME_CYCLE:
        if (!m_cover_detector.is_model_supported) {
          m_options.mode = CEPTON_SDK_FRAME_TIMED;
          m_options.length = 0.1f;
        }        
        break;
    }

    switch (m_options.mode) {
      case CEPTON_SDK_FRAME_TIMED:
        if (!m_options.length)
          return SensorError(CEPTON_ERROR_INVALID_ARGUMENTS,
                             "Frame length not set!");
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
        return true;
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
  // Outputs
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
      : m_frame_detector(sensor_info) {
    m_stride = sensor_info.return_count * sensor_info.segment_count;
    clear_impl();
  }

  FrameOptions get_options() const {
    LockGuard lock(m_mutex);
    return m_frame_detector.get_options();
  }

  SensorError set_options(const FrameOptions &options) {
    LockGuard lock(m_mutex);
    auto error_code = m_frame_detector.set_options(options);
    clear_impl();
    return error_code;
  }

  void clear() {
    LockGuard lock(m_mutex);
    clear_impl();
  }

  void add_points(int n_points, const SensorImagePoint *const image_points) {
    LockGuard lock(m_mutex);

    if (m_frame_detector.get_options().mode == CEPTON_SDK_FRAME_STREAMING) {
      callback(n_points, image_points);
      return;
    }

    assert(n_points % m_stride == 0);
    const int i_0 = (int)m_image_points.size();
    m_image_points.insert(m_image_points.end(), image_points,
                          image_points + n_points);
    for (int i = i_0; i < static_cast<int>(m_image_points.size());
         i += m_stride) {
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
  mutable std::timed_mutex m_mutex;
  int m_stride;
  std::vector<SensorImagePoint> m_image_points;

  int m_i_frame;
  FrameDetector m_frame_detector;
};

// -----------------------------------------------------------------------------
// Filter
// -----------------------------------------------------------------------------
/// Finds stray points caused by measurement noise.
/**
 * Marks point stray if its distance is different from neighbor points.
 */
class StrayFilter {
 public:
  StrayFilter() = default;

  StrayFilter(int segment_count, int return_count) {
    init(segment_count, return_count);
  }

  StrayFilter(const cepton_sdk::SensorInformation &sensor_info) {
    init(sensor_info);
  }

  void init(int segment_count, int return_count) {
    m_segment_count = segment_count;
    m_return_count = return_count;
  }

  void init(const cepton_sdk::SensorInformation &sensor_info) {
    init(sensor_info.segment_count, sensor_info.return_count);
  }

  void run(int n_points, cepton_sdk::SensorImagePoint *const c_image_points) {
    static thread_local std::vector<int> indices;
    const int stride = m_segment_count * m_return_count;
    for (int i_segment = 0; i_segment < m_segment_count; ++i_segment) {
      // Find valid indices for segment
      indices.clear();
      const int i_0 = i_segment * m_return_count;
      for (int i = i_0; i < n_points; i += stride) {
        const auto &image_point = c_image_points[i];
        if (!image_point.valid) continue;
        indices.push_back(i);
      }

      // Compute stray
      for (int i = 0; i < static_cast<int>(indices.size()); ++i) {
        for (int i_return = 0; i_return < m_return_count; ++i_return) {
          auto &image_point = c_image_points[indices[i] + i_return];
          if (!image_point.valid) continue;
          const int i_start = std::max<int>(i - n_neighbors, 0);
          const int i_end =
              std::min<int>(i + n_neighbors + 1, (int)indices.size());
          bool valid = false;
          for (int i_neighbor = i_start; i_neighbor < i_end; ++i_neighbor) {
            if (i_neighbor == i) continue;
            for (int i_return_neighbor = 0; i_return_neighbor < m_return_count;
                 ++i_return_neighbor) {
              const auto &other_point =
                  c_image_points[indices[i_neighbor] + i_return_neighbor];
              if (check_impl(image_point, other_point)) {
                valid = true;
                break;
              }
            }
          }
          image_point.valid = valid;
        }
      }
    }
  }

 private:
  bool check_impl(const cepton_sdk::SensorImagePoint &image_point,
                  const cepton_sdk::SensorImagePoint &other_point) {
    if (!other_point.valid) return false;
    const float distance_offset =
        std::abs(image_point.distance - other_point.distance);
    return (distance_offset < max_distance_offset);
  }

 public:
  // Options
  int n_neighbors = 2;
  float max_distance_offset = 10.0f;

 private:
  int m_segment_count;
  int m_return_count;
};

}  // namespace util
}  // namespace cepton_sdk

#include "cepton_sdk_impl/organizer.hpp"
