/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C++ utilities for rapid prototyping. API not guaranteed to
  be stable.
*/
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
#include <memory>
#include <queue>

namespace cepton_sdk {
namespace util {

//------------------------------------------------------------------------------
// Common
//------------------------------------------------------------------------------
/// Converts seconds -> microseconds.
inline int64_t to_usec(float sec) { return int64_t(sec * 1e6); }
/// Converts microseconds -> seconds.
inline float from_usec(int64_t usec) { return float(usec) * 1e-6f; }
/// # of microseconds in second.
const int64_t second_usec(to_usec(1.0f));
/// # of microseconds in hour.
const int64_t hour_usec(to_usec(60.0f * 60.0f));
/// # of seconds in hour.
const int64_t hour_sec(3600);

static const float PI = 3.14159265359f;
/// Converts radians -> degrees.
inline float to_degrees(float val) { return val * (180.0f / PI); }
/// Converts degrees -> radians.
inline float to_radians(float val) { return val * (PI / 180.0f); }

/// Returns `x` squared.
template <typename T>
inline T square(T x) {
  return x * x;
}

/// Returns value in range [0, n).
template <typename T>
T positive_modulo(const T &value, const T &n) {
  if (std::is_integral<T>::value) {
    T result = value % n;
    if (result < 0) result += n;
    return result;
  } else {
    return value - n * std::floor(value / n);
  }
}

/// Returns current unix timestamp [microseconds].
/**
 * This is the timestamp format used by all sdk functions.
 */
inline int64_t get_timestamp_usec() {
  const auto t_epoch = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(t_epoch).count();
}

class RAII {
 public:
  RAII() = default;
  RAII(const std::function<void()> &f)
      : m_raii(new bool(), [this, f](bool *const ptr_tmp) {
          delete ptr_tmp;
          f();
        }) {}

 private:
  std::shared_ptr<void> m_raii;
};

// -----------------------------------------------------------------------------
// Concurrent
// -----------------------------------------------------------------------------
/// Similar to `std::lock_guard`, but throws assert on deadlock.
class LockGuard {
 public:
  explicit LockGuard(std::timed_mutex &mutex);
  ~LockGuard();

  LockGuard(const LockGuard &) = delete;
  LockGuard &operator=(const LockGuard &) = delete;

 private:
  bool m_is_locked = false;
  std::timed_mutex &m_mutex;
};

/// Object pool for storing large reusable temporary objects.
/**
 * Internal use only.
 */
template <typename T>
class LargeObjectPool
    : public std::enable_shared_from_this<LargeObjectPool<T>> {
 public:
  std::shared_ptr<T> get();

 private:
  std::timed_mutex m_mutex;
  std::list<T> m_objects;
  std::vector<T *> m_free;
};

/// Multithreaded queue.
/**
 * Internal use only.
 */
template <typename T>
class SingleConsumerQueue {
 public:
  int size() const;
  bool empty() const;

  void clear();
  int push(const std::shared_ptr<T> &value, int max_size = 0);
  std::shared_ptr<T> pop(float timeout = 0.0f);

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
 *
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
  azimuth = std::atan2(1.0f, -image_x);
  inclination =
      std::acos(-image_z / std::sqrt(square(image_x) + square(image_z) + 1.0f));
}

/// 3d point class.
/**
 * Can't subclass from SensorImagePoint, needs to be POD.
 */
struct SensorPoint {
  int64_t timestamp;                   ///< Unix time [microseconds].
  float image_x;                       ///< x image coordinate.
  float distance;                      ///< Distance [meters].
  float image_z;                       ///< z image coordinate.
  float intensity;                     ///< Diffuse reflectance.
  CeptonSensorReturnType return_type;  ///< Strongest or farthest return.

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
    /// Bit flags.
    uint8_t flags;
    struct {
      /// If `false`, then the distance and intensity are invalid.
      uint8_t valid : 1;
      /// If `true`, then the intensity is invalid. Also, the distance is valid,
      /// but inaccurate.
      uint8_t saturated : 1;
    };
  };
#endif

  float x;  ///< x cartesian coordinate
  float y;  ///< y cartesian coordinate
  float z;  ///< z cartesian coordinate
};

/// Convenience method to convert `cepton_sdk::SensorImagePoint` to
/// `cepton_sdk::SensorPoint`.
inline void convert_sensor_image_point_to_point(
    const SensorImagePoint &image_point, SensorPoint &point) {
  *(SensorImagePoint *)(&point) = image_point;
#define COPY(name) point.name = image_point.name;
  COPY(timestamp);
  COPY(image_x);
  COPY(distance);
  COPY(image_z);
  COPY(intensity);
  COPY(return_type);
  COPY(flags);
#undef COPY

  convert_image_point_to_point(image_point.image_x, image_point.image_z,
                               image_point.distance, point.x, point.y, point.z);
}

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
  void clear();

  /// Register std::function.
  /**
   * @param func Callback function.
   * @param id Identifier used for `unlisten`.
   */
  SensorError listen(const std::function<void(TArgs...)> &func,
                     uint64_t *const id = nullptr);
  /// Register instance member function.
  /**
   * @param instance Parent class instance pointer.
   * @param func Callback function pointer.
   * @param id Identifier used for `unlisten`.
   */
  template <typename TClass>
  SensorError listen(TClass *const instance,
                     MemberFunction<TClass, TArgs...> func,
                     uint64_t *const id = nullptr);
  /// Unregister function.
  /**
   * @param id Identifier returned by `listen`.
   */
  SensorError unlisten(uint64_t id);

  /// Emit callback.
  /**
   * Calls all registered functions with `args`.
   */
  void operator()(TArgs... args) const;

  /// Used for registering as c callback.
  static void global_on_callback(TArgs... args, void *const instance);

 private:
  mutable std::timed_mutex m_mutex;
  int m_i_callback = 0;
  std::map<uint64_t, std::function<void(TArgs...)>> m_functions;
};

// -----------------------------------------------------------------------------
// Frame
// -----------------------------------------------------------------------------
enum _ExtremaType {
  EXTREMA_MIN = 0,
  EXTREMA_MAX = 1,
};
using ExtremaType = int32_t;

/// Detects single type extrema.
template <typename TData = bool>
class MaxDetector {
 public:
  struct Result {
    bool is_valid = false;
    ExtremaType type;
    int64_t timestamp = -1;
    float value;
    TData data;
  };

 public:
  /// Currently best max (not confirmed).
  const Result &result() const;
  /// Previously detected max.
  const Result &previous_result() const;

  void reset();

  bool update(int64_t timestamp, float value, const TData &data = TData());

 public:
  ExtremaType type = EXTREMA_MAX;
  int n_threshold = 0;
  float value_threshold = 0.0f;

 private:
  int64_t m_i_before = -1;
  int64_t m_i_after = -1;
  Result m_result;
  Result m_previous_result;
};

/// Detects alternating type extrema.
template <typename TData = bool>
class ExtremaDetector {
 public:
  using Result = typename MaxDetector<TData>::Result;

 public:
  ExtremaDetector();

  /// Previously detected extrema.
  const Result &previous_result() const;
  /// Both previously detected extrema (indexed by type).
  const std::vector<Result> &previous_results() const;

  void reset();
  bool update(int64_t idx, float value, const TData &data = TData());

 public:
  int n_threshold = 1;
  float value_threshold = 0.0f;

 private:
  int m_type = 0;
  MaxDetector<TData> m_filter;
  Result m_previous_result;
  std::vector<Result> m_previous_results{2};
};

namespace internal {

template <typename TData = bool>
class FrameDetectorBase {
 public:
  struct Result {
    bool is_valid = false;
    /// Frame type (dependent on sensor model).
    int type = 0;
    /// Frame end timestamp.
    int64_t timestamp = -1;
    /// Extra data.
    TData data;
  };

 public:
  FrameDetectorBase() = default;
  virtual ~FrameDetectorBase() = default;

  int n_types() const;

  /// Returns last detected frame.
  const Result &previous_result() const;
  /// Returns all previous frames indexed by type.
  const std::vector<Result> &previous_results() const;

  /// Returns frame period in seconds.
  /**
   * Invalid if negative.
   * Next frame will be at `previous_result().timestamp + to_usec(period())`.
   */
  float period() const;

  /// Resets detector.
  /**
   * Not necessary to call after successful detection.
   */
  virtual void reset();
  /// Adds new point and returns whether new frame is detected.
  /**
   * New frame can be queried with `previous_result`.
   */
  virtual bool update(const SensorImagePoint &point,
                      const TData &data = TData()) = 0;

 protected:
  explicit FrameDetectorBase(int n_types);
  void init_types(int n_types);

 protected:
  bool finalize(const Result &result);

 public:
  /// If true, only record type `0` results.
  bool enable_cycle = false;

 protected:
  Result m_previous_result;
  std::vector<Result> m_previous_results;
  float m_period = -1.0f;
};

/// Detects frames for Sora sensor models (fast x scan).
template <typename TData = bool>
class SoraFrameDetector : public FrameDetectorBase<TData> {
 public:
  SoraFrameDetector();

  void reset() override;
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 private:
  ExtremaDetector<TData> m_detector;
};

/// Detects frames for HR80 sensor models (slow x scan, fast z scan).
template <typename TData = bool>
class HR80FrameDetector : public FrameDetectorBase<TData> {
 public:
  HR80FrameDetector();

  void reset() override;
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 private:
  ExtremaDetector<std::pair<float, TData>> m_fast_detector;
  ExtremaDetector<TData> m_slow_detector;
};

/// Detects frames for Vista sensor models (fast x/z scan).
template <typename TData = bool>
class VistaFrameDetector : public FrameDetectorBase<TData> {
 public:
  VistaFrameDetector();

  void reset() override;
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 private:
  std::vector<ExtremaDetector<TData>> m_detectors;
  MaxDetector<TData> m_offset_detector;
  MaxDetector<TData> m_phase_detector;
  int m_type = 0;
};

/// Generates frames at periodic time intervals.
template <typename TData = bool>
class TimedFrameDetector : public FrameDetectorBase<TData> {
 public:
  TimedFrameDetector(float frame_length);

  void reset() override;
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 public:
  float frame_length;

 private:
  int64_t m_t = 0;
};

}  // namespace internal

/// Detects scanlines (change in image x coordinate).
template <typename TData = bool>
class ScanlineDetector : public internal::FrameDetectorBase<TData> {
 public:
  ScanlineDetector(const SensorInformation &sensor_info);

  void reset() override;
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 private:
  ExtremaDetector<> m_detector;
};

/// Detects frames in streaming sensor data.
/**
 * `Result::type`
 * - Sora: 0=left-right, 1=right-left
 * - HR80: 0=left-right, 1=right-left
 * - Vista: undefined
 */
template <typename TData = bool>
class FrameDetector : public internal::FrameDetectorBase<TData> {
 public:
  /// FrameDetector class constructor passing in SensorInformation.
  FrameDetector(const SensorInformation &sensor_info);

  /// Returns frame options for FrameDetector.
  const FrameOptions &get_options() const;
  /// Set frame options
  SensorError set_options(const FrameOptions &options);

  /// Completely resets detector.
  /**
   * Only use if also clearing points accumulator.
   */
  void reset() override;
  /// Returns true if frame found.
  /**
   * Automatically resets after frame is found.
   */
  bool update(const SensorImagePoint &point,
              const TData &data = TData()) override;

 private:
  internal::FrameDetectorBase<TData> *detector();

 private:
  FrameOptions m_options = cepton_sdk_create_frame_options();

  std::unique_ptr<internal::FrameDetectorBase<TData>> m_cover_detector;
  internal::TimedFrameDetector<TData> m_timed_detector;
};

/// Accumulates image points, and emits frames to callback.
/**
 * See `samples/frame.cpp`.
 */
class FrameAccumulator {
 public:
  /// FrameAccumulator class constructor passing in SensorInformation.
  FrameAccumulator(const SensorInformation &sensor_info);

  /// Return frame options for FrameAccumulator.
  FrameOptions get_options() const;
  /// Set options for FrameAccumulator
  SensorError set_options(const FrameOptions &options);

  void clear();
  void add_points(std::size_t n_points,
                  const SensorImagePoint *const image_points);

 private:
  void clear_impl();

 public:
  /// Frames callback
  Callback<std::size_t, const SensorImagePoint *> callback;

 private:
  const int m_stride;

  mutable std::timed_mutex m_mutex;
  int64_t m_idx_0 = 0;
  int64_t m_idx = -1;
  int64_t m_i_frame = -1;
  FrameDetector<int64_t> m_frame_detector;
  std::vector<SensorImagePoint> m_image_points;
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
  StrayFilter(int segment_count, int return_count);
  StrayFilter(const cepton_sdk::SensorInformation &sensor_info);

  void init(int segment_count, int return_count);
  void init(const cepton_sdk::SensorInformation &sensor_info);
  void run(int n_points, cepton_sdk::SensorImagePoint *const c_image_points);

 private:
  bool check_impl(const cepton_sdk::SensorImagePoint &image_point,
                  const cepton_sdk::SensorImagePoint &other_point);

 public:
  /// Minimum number of adjacent neighbors for point to be not stray.
  int n_neighbors = 2;
  /// Maximum distance offset to consider points adjacent.
  float max_distance_offset = 10.0f;

 private:
  int m_segment_count = 1;
  int m_return_count = 1;
};

}  // namespace util
}  // namespace cepton_sdk

#include "cepton_sdk_impl/cepton_sdk_util.inc"
#include "cepton_sdk_impl/organizer.hpp"
