/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C++ interface.
*/
#include "cepton_sdk_util.hpp"

namespace cepton_sdk {
namespace util {
// -----------------------------------------------------------------------------
// LockGuard
// -----------------------------------------------------------------------------
inline LockGuard::LockGuard(std::timed_mutex &mutex) : m_mutex(mutex) {
  m_is_locked = m_mutex.try_lock_for(std::chrono::seconds(1));
  CEPTON_ASSERT(m_is_locked, "Deadlock!");
}

inline LockGuard::~LockGuard() {
  if (m_is_locked) m_mutex.unlock();
}

// -----------------------------------------------------------------------------
// LargeObjectPool
// -----------------------------------------------------------------------------
template <typename T>
std::shared_ptr<T> LargeObjectPool<T>::get() {
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

// -----------------------------------------------------------------------------
// SingleConsumerQueue
// -----------------------------------------------------------------------------
template <typename T>
int SingleConsumerQueue<T>::size() const {
  return m_size;
}

template <typename T>
bool SingleConsumerQueue<T>::empty() const {
  return size() == 0;
}

template <typename T>
void SingleConsumerQueue<T>::clear() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_queue = std::queue<std::shared_ptr<T>>();
  m_size = (int)m_queue.size();
}

template <typename T>
int SingleConsumerQueue<T>::push(const std::shared_ptr<T> &value,
                                 int max_size) {
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

template <typename T>
std::shared_ptr<T> SingleConsumerQueue<T>::pop(float timeout) {
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

// -----------------------------------------------------------------------------
// Organizer
// -----------------------------------------------------------------------------
inline Organizer::Organizer(cepton_sdk::SensorInformation sensor_info) {
  m_settings.vertical_range_radians = to_radians(30.f);
  switch (sensor_info.model) {
    case HR80W:
    case VISTA_860_GEN2:
    case VISTA_P60:
      m_settings.horizontal_range_radians = to_radians(70.f);
      break;
    case VISTA_X120:
      m_settings.horizontal_range_radians = to_radians(120.f);
      m_settings.vertical_range_radians = to_radians(18.f);
      break;
    case VISTA_X15:
      m_settings.horizontal_range_radians = to_radians(15.f);
      m_settings.vertical_range_radians = to_radians(18.f);
      break;
    case VISTA_T30:
      m_settings.horizontal_range_radians = to_radians(30.f);
      m_settings.vertical_range_radians = to_radians(18.f);
      break;
    case HR80T_R2:
    default:
      m_settings.horizontal_range_radians = to_radians(40.f);
  }
}

inline void Organizer::organize_points(
    const int num_points_in, const int n_returns,
    const CeptonSensorImagePoint *const unorganized_points,
    cepton_sdk::util::OrganizedCloud &organized_cloud) {
  std::lock_guard<std::mutex> lock(m_organizer_mutex);

  // set start and end timestamps to numeric limits
  organized_cloud.timestamp_start = std::numeric_limits<long>::max();
  organized_cloud.timestamp_end = std::numeric_limits<long>::min();

  organized_cloud.width =
      static_cast<int>(m_settings.horizontal_range_radians /
                       m_settings.horizontal_bin_size_radians);
  organized_cloud.height = static_cast<int>(
      m_settings.vertical_range_radians / m_settings.vertical_bin_size_radians);
  organized_cloud.n_returns = n_returns;

  int cloud_size = organized_cloud.width * organized_cloud.height * n_returns;

  // initialize points
  organized_cloud.points.resize(cloud_size);

  // initialize info cells
  organized_cloud.info_cells.clear();
  organized_cloud.info_cells.resize(cloud_size);

  for (int point_index = 0; point_index < num_points_in; point_index++) {
    const auto &unorganized_point = unorganized_points[point_index];
    const auto grid_index = getGridIndex(
        organized_cloud, unorganized_point.image_x, unorganized_point.image_z);

    int n_return = 0;
    if (unorganized_point.return_type | CEPTON_RETURN_STRONGEST) {
      n_return = 0;
    } else if (n_returns > 1) {
      n_return = 1;
    }

    if (grid_index.row >= 0 && grid_index.col >= 0) {
      const size_t organized_index = static_cast<size_t>(
          organized_cloud.getIndex(grid_index.row, grid_index.col, n_return));

      auto &organized_point = organized_cloud.points[organized_index];
      auto &info_cell = organized_cloud.info_cells[organized_index];

      // If the existing cell is unoccupied populate. If we have a valid point
      // and it's more recent we always want to populate with it.
      if (!info_cell.occupied_cell ||
          (unorganized_point.valid &&
           organized_point.timestamp < unorganized_point.timestamp)) {
        organized_cloud.timestamp_start = std::min(
            organized_cloud.timestamp_start, unorganized_point.timestamp);
        organized_cloud.timestamp_end = std::max(organized_cloud.timestamp_end,
                                                 unorganized_point.timestamp);
        info_cell.occupied_cell = true;
        info_cell.original_index = point_index;

        // We always populated everything except the image_x and image_z
        organized_point.timestamp = unorganized_point.timestamp;
        organized_point.distance = unorganized_point.distance;
        organized_point.intensity = unorganized_point.intensity;
        organized_point.return_type = unorganized_point.return_type;
        organized_point.valid = unorganized_point.valid;
        organized_point.saturated = unorganized_point.saturated;
        organized_point.segment_id = unorganized_point.segment_id;

        // In RECENT mode, populate the image xy points with the unorganized
        // point for accuracy.
        if (m_settings.mode == OrganizerMode::RECENT) {
          organized_point.image_x = unorganized_point.image_x;
          organized_point.image_z = unorganized_point.image_z;
        } else {
          auto image_xz = getXZ(organized_cloud.width, organized_cloud.height,
                                grid_index.row, grid_index.col);
          organized_point.image_x = image_xz.X;
          organized_point.image_z = image_xz.Z;
        }
      }
    }
  }
  for (int row = 0; row < organized_cloud.height; ++row) {
    for (int col = 0; col < organized_cloud.width; ++col) {
      for (int n_return = 0; n_return < organized_cloud.n_returns; ++n_return) {
        int index = organized_cloud.getIndex(row, col, n_return);
        auto &pt = organized_cloud.points[index];
        if (pt.distance == 0.f) {
          ImageXZ image_xz =
              getXZ(organized_cloud.width, organized_cloud.height, row, col);
          pt.image_x = image_xz.X;
          pt.image_z = image_xz.Z;
        }
      }
    }
  }
}

inline void Organizer::mode(Organizer::OrganizerMode mode) {
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings.mode = std::move(mode);
}

inline void Organizer::binSize(float bin_size) {
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings.horizontal_bin_size_radians = bin_size;
  m_settings.vertical_bin_size_radians = bin_size;
}

inline Organizer::OrganizerSettings Organizer::settings() { return m_settings; }

inline void Organizer::settings(OrganizerSettings organizer_settings) {
  std::lock_guard<std::mutex> lock(m_organizer_mutex);
  m_settings = organizer_settings;
}

inline Organizer::ImageXZ Organizer::getXZ(int width, int height, int row,
                                           int col) {
  float angle_horizontal = (m_settings.horizontal_range_radians / width) *
                           ((col + 0.5f) - width / 2);
  float angle_vertical = (m_settings.vertical_range_radians / height) *
                         ((row + 0.5f) - height / 2);
  return {tanf(angle_horizontal), tanf(angle_vertical)};
}

inline Organizer::GridIndex Organizer::getGridIndex(const OrganizedCloud &cloud,
                                                    float X, float Z) {
  float angle_horizontal = atanf(X);
  float angle_vertical = atanf(Z);
  GridIndex index;

  index.col =
      cloud.width / 2 + static_cast<int>(cloud.width * angle_horizontal /
                                         m_settings.horizontal_range_radians);
  index.row =
      cloud.height / 2 + static_cast<int>(cloud.height * angle_vertical /
                                          m_settings.vertical_range_radians);
  if (index.col < 0 || index.col >= cloud.width) {
    // Data outside of expected range given invalid index
    index.col = -1;
  }

  if (index.row < 0 || index.row >= cloud.height) {
    // Data outside of expected range given invalid index
    index.row = -1;
  }
  return index;
}

// -----------------------------------------------------------------------------
// Callback
// -----------------------------------------------------------------------------
template <typename... TArgs>
void Callback<TArgs...>::clear() {
  LockGuard lock(m_mutex);
  m_i_callback = 0;
  m_functions.clear();
}

/// Register std::function.
template <typename... TArgs>
SensorError Callback<TArgs...>::listen(
    const std::function<void(TArgs...)> &func, uint64_t *const id) {
  LockGuard lock(m_mutex);
  if (id) *id = m_i_callback;
  m_functions[m_i_callback] = func;
  ++m_i_callback;
  return CEPTON_SUCCESS;
}

template <typename... TArgs>
RAII Callback<TArgs...>::listen_auto(
    const std::function<void(TArgs...)> &func) {
  uint64_t id;
  CEPTON_CHECK_ERROR(listen(func, &id));
  return RAII([this, id]() { CEPTON_CHECK_ERROR(unlisten(id)); });
}

template <typename... TArgs>
template <typename TClass>
SensorError Callback<TArgs...>::listen(TClass *const instance,
                                       MemberFunction<TClass, TArgs...> func,
                                       uint64_t *const id) {
  return listen([instance, func](TArgs... args) { (instance->*func)(args...); },
                id);
}

template <typename... TArgs>
template <typename TClass>
RAII Callback<TArgs...>::listen_auto(TClass *const instance,
                                     MemberFunction<TClass, TArgs...> func) {
  uint64_t id;
  CEPTON_CHECK_ERROR(listen(
      [instance, func](TArgs... args) { (instance->*func)(args...); }, &id));
  return RAII([this, id]() { CEPTON_CHECK_ERROR(unlisten(id)); });
}

template <typename... TArgs>
SensorError Callback<TArgs...>::unlisten(uint64_t id) {
  LockGuard lock(m_mutex);
  CEPTON_ASSERT_ERROR(m_functions.count(id), CEPTON_ERROR_INVALID_ARGUMENTS,
                      "Invalid function id!");
  m_functions.erase(id);
  return CEPTON_SUCCESS;
}

template <typename... TArgs>
void Callback<TArgs...>::operator()(TArgs... args) const {
  LockGuard lock(m_mutex);
  for (const auto &iter : m_functions) {
    const auto &func = iter.second;
    func(args...);
  }
}

template <typename... TArgs>
void Callback<TArgs...>::global_on_callback(TArgs... args,
                                            void *const instance) {
  ((Callback *)instance)->operator()(args...);
}

// -----------------------------------------------------------------------------
// MaxDetector
// -----------------------------------------------------------------------------
template <typename TData>
const typename MaxDetector<TData>::Result &MaxDetector<TData>::result() const {
  return m_result;
}

template <typename TData>
const typename MaxDetector<TData>::Result &MaxDetector<TData>::previous_result()
    const {
  return m_previous_result;
}

template <typename TData>
void MaxDetector<TData>::reset() {
  m_i_before = -1;
  m_i_after = -1;
  m_result = Result();
  m_previous_result = Result();
}

template <typename TData>
bool MaxDetector<TData>::update(int64_t timestamp, float value,
                                const TData &data) {
  if (!m_result.is_valid && (m_result.timestamp >= 0) &&
      (m_i_after >= n_threshold)) {
    m_i_before = -1;
    m_result = Result();
  }
  ++m_i_before;
  const float sign = (type == 0) ? -1.0f : 1.0f;
  if ((m_result.timestamp < 0) || ((sign * (value - m_result.value)) > 0.0f)) {
    m_i_after = -1;
    m_result.is_valid = m_i_before >= n_threshold;
    m_result.type = type;
    m_result.timestamp = timestamp;
    m_result.value = value;
    m_result.data = data;
  }
  ++m_i_after;
  if (!n_threshold && !value_threshold) return false;
  if (m_i_after < n_threshold) return false;
  if ((sign * (m_result.value - value)) < value_threshold) return false;
  m_i_before = -1;

  m_previous_result = m_result;
  m_result = Result();
  return m_previous_result.is_valid;
}

// -----------------------------------------------------------------------------
// ExtremaDetector
// -----------------------------------------------------------------------------
template <typename TData>
ExtremaDetector<TData>::ExtremaDetector() : m_previous_results(2) {}

template <typename TData>
const typename ExtremaDetector<TData>::Result &
ExtremaDetector<TData>::previous_result() const {
  return m_previous_result;
}

template <typename TData>
const std::vector<typename ExtremaDetector<TData>::Result>
    &ExtremaDetector<TData>::previous_results() const {
  return m_previous_results;
}

template <typename TData>
void ExtremaDetector<TData>::reset() {
  m_filter.reset();
  m_previous_result = Result();
  std::fill(m_previous_results.begin(), m_previous_results.end(), Result());
}

template <typename TData>
bool ExtremaDetector<TData>::update(int64_t timestamp, float value,
                                    const TData &data) {
  m_filter.n_threshold = n_threshold;
  m_filter.value_threshold = 0.0f;
  if (!m_filter.update(timestamp, value, data)) return false;
  const auto &result = m_filter.previous_result();
  m_filter.type = (m_filter.type + 1) % 2;

  m_previous_result = result;
  m_previous_results[result.type] = result;
  return true;
}

namespace internal {
// -----------------------------------------------------------------------------
// FrameDetectorBase
// -----------------------------------------------------------------------------
template <typename TData>
FrameDetectorBase<TData>::FrameDetectorBase(int n_types_) {
  init_types(n_types_);
}

template <typename TData>
void FrameDetectorBase<TData>::init_types(int n_types_) {
  m_previous_results.resize(n_types_);
  std::fill(m_previous_results.begin(), m_previous_results.end(), Result());
}

template <typename TData>
int FrameDetectorBase<TData>::n_types() const {
  return (int)m_previous_results.size();
}

template <typename TData>
const typename FrameDetectorBase<TData>::Result &
FrameDetectorBase<TData>::previous_result() const {
  return m_previous_result;
}

template <typename TData>
const std::vector<typename FrameDetectorBase<TData>::Result>
    &FrameDetectorBase<TData>::previous_results() const {
  return m_previous_results;
}

template <typename TData>
float FrameDetectorBase<TData>::period() const {
  return m_period;
}

template <typename TData>
void FrameDetectorBase<TData>::reset() {
  m_previous_result = Result();
  std::fill(m_previous_results.begin(), m_previous_results.end(), Result());
  m_period = -1.0f;
}

template <typename TData>
bool FrameDetectorBase<TData>::finalize(const Result &result) {
  m_previous_results.at(result.type) = result;
  if (enable_cycle && (result.type != 0)) return false;
  if (m_previous_result.is_valid)
    m_period = from_usec(result.timestamp - m_previous_result.timestamp);
  m_previous_result = result;
  return true;
}

// -----------------------------------------------------------------------------
// SoraFrameDetector
// -----------------------------------------------------------------------------
template <typename TData>
SoraFrameDetector<TData>::SoraFrameDetector() : FrameDetectorBase<TData>(2) {
  m_detector.n_threshold = 5;
}

template <typename TData>
void SoraFrameDetector<TData>::reset() {
  FrameDetectorBase<TData>::reset();
  m_detector.reset();
}

template <typename TData>
bool SoraFrameDetector<TData>::update(const SensorImagePoint &point,
                                      const TData &data) {
  if (!m_detector.update(point.timestamp, point.image_x, data)) return false;
  const auto &result_tmp = m_detector.previous_result();

  typename FrameDetectorBase<TData>::Result result;
  result.is_valid = true;
  result.type = result_tmp.type;
  result.timestamp = result_tmp.timestamp;
  result.data = result_tmp.data;
  return this->finalize(result);
}

// -----------------------------------------------------------------------------
// HR80FrameDetector
// -----------------------------------------------------------------------------
template <typename TData>
HR80FrameDetector<TData>::HR80FrameDetector() : FrameDetectorBase<TData>(2) {
  m_fast_detector.n_threshold = 5;
  m_slow_detector.n_threshold = 4;
}

template <typename TData>
void HR80FrameDetector<TData>::reset() {
  FrameDetectorBase<TData>::reset();
  m_fast_detector.reset();
  m_slow_detector.reset();
}

template <typename TData>
bool HR80FrameDetector<TData>::update(const SensorImagePoint &point,
                                      const TData &data) {
  if (!m_fast_detector.update(point.timestamp, point.image_z,
                              {point.image_x, data}))
    return false;
  const auto &fast_result = m_fast_detector.previous_result();
  if (!m_slow_detector.update(fast_result.timestamp, fast_result.data.first,
                              fast_result.data.second))
    return false;
  const auto &result_tmp = m_slow_detector.previous_result();

  typename FrameDetectorBase<TData>::Result result;
  result.is_valid = true;
  result.type = result_tmp.type;
  result.timestamp = result_tmp.timestamp;
  result.data = result_tmp.data;
  return this->finalize(result);
}

// -----------------------------------------------------------------------------
// VistaFrameDetector
// -----------------------------------------------------------------------------
template <typename TData>
VistaFrameDetector<TData>::VistaFrameDetector()
    : FrameDetectorBase<TData>(2), m_detectors(2) {
  for (int i = 0; i < 2; ++i) {
    m_detectors[i].n_threshold = 5;
  }
  m_offset_detector.type = EXTREMA_MIN;
  m_phase_detector.type = EXTREMA_MAX;
  m_phase_detector.n_threshold = 2;
}

template <typename TData>
void VistaFrameDetector<TData>::reset() {
  FrameDetectorBase<TData>::reset();
  for (int i = 0; i < 2; ++i) {
    m_detectors[i].reset();
  }
  m_offset_detector.reset();
  m_phase_detector.reset();
}

template <typename TData>
bool VistaFrameDetector<TData>::update(const SensorImagePoint &point,
                                       const TData &data) {
  std::array<float, 2> position = {{point.image_x, point.image_z}};

  // Compute image bounds.
  std::array<bool, 2> has_new_scanline;
  for (int i = 0; i < 2; ++i) {
    has_new_scanline[i] =
        m_detectors[i].update(point.timestamp, position[i], data);
  }
  for (int i = 0; i < 2; ++i) {
    std::array<float, 2> bounds;
    for (int i_bound = 0; i_bound < 2; ++i_bound) {
      const auto &result_tmp = m_detectors[i].previous_results()[i_bound];
      if (!result_tmp.is_valid) return false;
      bounds[i_bound] = result_tmp.value;
    }
    // Center and normalize positions.
    const float center = 0.5f * (bounds[0] + bounds[1]);
    const float ptp = bounds[1] - bounds[0];
    position[i] = (position[i] - center) / (0.5f * ptp);
  }

  // Compute offset.
  const float offset = square(position[0]) + square(position[1]);
  m_offset_detector.update(point.timestamp, offset, data);
  if (!has_new_scanline[0]) return false;
  const typename decltype(m_offset_detector)::Result offset_result =
      m_offset_detector.result();
  m_offset_detector.reset();

  // Try to detect frame boundary at consistent point
  const auto &x_result = m_detectors[0].previous_results()[m_type];
  if (!m_phase_detector.update(x_result.timestamp, offset_result.value,
                               x_result.data))
    return false;
  const auto &result_tmp = m_phase_detector.previous_result();

  typename FrameDetectorBase<TData>::Result result;
  result.is_valid = true;
  result.type = m_type;
  m_type = (m_type + 1) % 2;
  result.timestamp = result_tmp.timestamp;
  result.data = result_tmp.data;
  return this->finalize(result);
}

// -----------------------------------------------------------------------------
// TimedFrameDetector
// -----------------------------------------------------------------------------
template <typename TData>
TimedFrameDetector<TData>::TimedFrameDetector(float frame_length_)
    : FrameDetectorBase<TData>(1), frame_length(frame_length_) {}

template <typename TData>
void TimedFrameDetector<TData>::reset() {
  FrameDetectorBase<TData>::reset();
  m_t = 0;
}

template <typename TData>
bool TimedFrameDetector<TData>::update(const SensorImagePoint &point,
                                       const TData &data) {
  if (m_t == 0) m_t = point.timestamp;
  const float t_diff = from_usec(point.timestamp - m_t);
  if ((t_diff < frame_length) && (t_diff > -0.01f)) return false;

  m_t = 0;

  typename FrameDetectorBase<TData>::Result result;
  result.is_valid = true;
  result.timestamp = point.timestamp;
  result.data = data;
  return this->finalize(result);
}
}  // namespace internal

// -----------------------------------------------------------------------------
// ScanlineDetector
// -----------------------------------------------------------------------------
template <typename TData>
ScanlineDetector<TData>::ScanlineDetector(const SensorInformation &sensor_info)
    : internal::FrameDetectorBase<TData>(2) {
  m_detector.n_threshold = 5;
}

template <typename TData>
void ScanlineDetector<TData>::reset() {
  internal::FrameDetectorBase<TData>::reset();
  m_detector.reset();
}

template <typename TData>
bool ScanlineDetector<TData>::update(const SensorImagePoint &point,
                                     const TData &data) {
  if (!m_detector.update(point.timestamp, point.image_z)) return false;
  const auto &result_tmp = m_detector.previous_result();

  typename internal::FrameDetectorBase<TData>::Result result;
  result.is_valid = true;
  result.type = result_tmp.type;
  result.timestamp = result_tmp.timestamp;
  result.data = result_tmp.data;
  return this->finalize(result);
}

// -----------------------------------------------------------------------------
// FrameDetector
// -----------------------------------------------------------------------------
template <typename TData>
FrameDetector<TData>::FrameDetector(const SensorInformation &sensor_info)
    : m_timed_detector(0.0f) {
  if (cepton_sdk::is_sora(sensor_info.model)) {
    m_cover_detector.reset(new internal::SoraFrameDetector<TData>());
  } else if (cepton_sdk::is_hr80(sensor_info.model)) {
    m_cover_detector.reset(new internal::HR80FrameDetector<TData>());
  } else if (cepton_sdk::is_vista(sensor_info.model)) {
    m_cover_detector.reset(new internal::VistaFrameDetector<TData>());
  }
}

template <typename TData>
internal::FrameDetectorBase<TData> *FrameDetector<TData>::detector() {
  switch (m_options.mode) {
    case CEPTON_SDK_FRAME_COVER:
    case CEPTON_SDK_FRAME_CYCLE:
      return m_cover_detector.get();
    case CEPTON_SDK_FRAME_TIMED:
      return &m_timed_detector;
    default:
      return nullptr;
  }
}

template <typename TData>
const FrameOptions &FrameDetector<TData>::get_options() const {
  return m_options;
}

template <typename TData>
SensorError FrameDetector<TData>::set_options(const FrameOptions &options) {
  m_options = options;
  if (!detector()) return CEPTON_SUCCESS;

  // Check
  switch (m_options.mode) {
    case CEPTON_SDK_FRAME_TIMED:
      CEPTON_ASSERT_ERROR(m_options.length, CEPTON_ERROR_INVALID_ARGUMENTS,
                          "Frame length not set!");
      m_timed_detector.frame_length = m_options.length;
      break;
  }

  switch (m_options.mode) {
    case CEPTON_SDK_FRAME_COVER:
      m_cover_detector->enable_cycle = false;
      break;
    case CEPTON_SDK_FRAME_CYCLE:
      m_cover_detector->enable_cycle = true;
      break;
    case CEPTON_SDK_FRAME_TIMED:
      m_timed_detector.frame_length = m_options.length;
      break;
  }
  this->init_types(detector()->n_types());
  reset();
  return CEPTON_SUCCESS;
}

template <typename TData>
void FrameDetector<TData>::reset() {
  internal::FrameDetectorBase<TData>::reset();
  if (detector()) detector()->reset();
}

template <typename TData>
bool FrameDetector<TData>::update(const SensorImagePoint &point,
                                  const TData &data) {
  CEPTON_ASSERT(detector(), "");
  if (!detector()->update(point, data)) return false;
  return this->finalize(detector()->previous_result());
}

template <typename TData>
void FrameDetector<TData>::set_frame_mode(CeptonSDKFrameMode mode) {
  m_options.mode = mode;
}

// -----------------------------------------------------------------------------
// FrameAccumulator
// -----------------------------------------------------------------------------
inline FrameAccumulator::FrameAccumulator(const SensorInformation &sensor_info)
    : m_stride(sensor_info.return_count * sensor_info.segment_count),
      m_frame_detector(sensor_info) {}

inline FrameOptions FrameAccumulator::get_options() const {
  LockGuard lock(m_mutex);
  return m_frame_detector.get_options();
}

inline SensorError FrameAccumulator::set_options(const FrameOptions &options) {
  LockGuard lock(m_mutex);
  clear_impl();
  return CEPTON_PROCESS_ERROR(m_frame_detector.set_options(options));
}

inline void FrameAccumulator::clear() {
  LockGuard lock(m_mutex);
  clear_impl();
}

inline void FrameAccumulator::clear_impl() {
  m_idx_0 = 0;
  m_idx = -1;
  m_i_frame = -1;
  m_frame_detector.reset();
  m_image_points.clear();
}

inline void FrameAccumulator::add_points(
    std::size_t n_points, const SensorImagePoint *const image_points) {
  LockGuard lock(m_mutex);

  if (m_frame_detector.get_options().mode == CEPTON_SDK_FRAME_STREAMING) {
    callback(n_points, image_points);
    return;
  }

  CEPTON_ASSERT((n_points % m_stride) == 0, "");
  const int i_0 = (int)m_image_points.size();
  m_image_points.insert(m_image_points.end(), image_points,
                        image_points + n_points);
  for (int i = i_0; i < (int)m_image_points.size(); i += m_stride) {
    ++m_idx;
    if (!m_frame_detector.update(m_image_points[i], m_idx)) continue;
    const auto &result = m_frame_detector.previous_result();

    ++m_i_frame;
    const int n = int(result.data + 1 - m_idx_0);
    m_idx_0 = result.data + 1;
    const int n_image = n * m_stride;
    if (m_i_frame >= 2) callback(n_image, m_image_points.data());
    m_image_points.erase(m_image_points.begin(),
                         m_image_points.begin() + n_image);
    i -= n_image;
  }
}
}  // namespace util
}  // namespace cepton_sdk