/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C++ interface.
*/
#include "cepton_sdk.hpp"

namespace cepton_sdk {

namespace internal {
inline std::string create_context_message(const std::string &file, int line,
                                          const std::string &code) {
  std::array<char, 1000> buffer;
  std::snprintf(buffer.data(), buffer.size(), "  File \"%s\", line %i\n    %s",
                file.c_str(), line, code.c_str());
  return std::string(buffer.data());
}

inline std::string create_assert_message(const std::string &file, int line,
                                         const std::string &code,
                                         const std::string &msg) {
  std::string result;
  result.append("AssertionError");
  if (!msg.empty()) {
    result.append(": ");
    result.append(msg);
  }
  result.append("\n");
  result.append(create_context_message(file, line, code));
  return result;
}

inline void throw_assert(const std::string &file, int line,
                         const std::string &code, const std::string &msg) {
  std::fprintf(stderr, "%s\n",
               create_assert_message(file, line, code, msg).c_str());
#ifdef CEPTON_ENABLE_EXCEPTIONS
  std::terminate();
#endif
}
}  // namespace internal

//------------------------------------------------------------------------------
// SensorError
//------------------------------------------------------------------------------
inline SensorError::SensorError(SensorErrorCode code_, const std::string &msg_)
    : std::runtime_error(create_message(code_, msg_)),
      m_code(code_),
      m_msg(msg_) {
  CEPTON_ASSERT(!get_error_code_name(code_).empty(),
                std::string("Invalid error code: ") + std::to_string(m_code));
}

inline SensorError::SensorError(SensorErrorCode code_)
    : SensorError(code_, "") {}

inline SensorError::SensorError() : SensorError(CEPTON_SUCCESS) {}

inline SensorError::~SensorError() { check_used(); }

inline SensorError::SensorError(const SensorError &other)
    : std::runtime_error(other) {
  m_code = other.code();
  m_msg = other.msg();
}

inline SensorError &SensorError::operator=(const SensorError &other) {
  check_used();
  std::runtime_error::operator=(other);
  m_code = other.code();
  m_msg = other.msg();
  m_used = false;
  return *this;
}

inline bool SensorError::used() const { return m_used; }

inline const SensorError &SensorError::ignore() const {
  m_used = true;
  return *this;
}

inline const char *SensorError::what() const noexcept {
  m_used = true;
  return std::runtime_error::what();
}

inline const std::string &SensorError::msg() const {
  m_used = true;
  return m_msg;
}

inline SensorErrorCode SensorError::code() const {
  m_used = true;
  return m_code;
}

inline SensorError::operator SensorErrorCode() const { return code(); }

inline SensorError::operator bool() const { return code(); }

inline const std::string SensorError::name() const {
  return get_error_code_name(code());
}

inline bool SensorError::is_error() const { return is_error_code(code()); }

inline bool SensorError::is_fault() const { return is_fault_code(code()); }

inline std::string SensorError::create_message(SensorErrorCode code,
                                               const std::string &msg) {
  if (!code) return "";
  std::string result = get_error_code_name(code);
  if (!msg.empty()) {
    result.append("\n");
    result.append(msg);
  }
  return result;
}

inline void SensorError::check_used() const {
  CEPTON_ASSERT(!m_code || m_used, std::string("Error not used: ") + what()); // NOLINT
}

namespace internal {
inline SensorError add_error_context(const SensorError &error,
                                     const std::string &context) {
  if (!error) return error;
  std::string msg;
  if (!error.msg().empty()) {
    msg.append(error.msg());
    msg.append("\n");
  }
  msg.append(context);
  return SensorError(error.code(), msg);
}
}  // namespace internal

//------------------------------------------------------------------------------
// SensorErrorWrapper
//------------------------------------------------------------------------------
inline SensorErrorWrapper::SensorErrorWrapper(const std::string &context_)
    : m_context(context_) {}

inline SensorErrorWrapper &SensorErrorWrapper::operator=(
    const SensorError &error_) {
  if (enable_accumulation && m_error) return *this;
#ifdef CEPTON_INTERNAL
  m_error = internal::add_error_context(error_, m_context);
#else
  m_error = error_;
#endif
  return *this;
}

inline SensorErrorWrapper::operator bool() const { return m_error; }

inline const SensorError &SensorErrorWrapper::error() const { return m_error; }

inline SensorErrorWrapper::operator const SensorError &() const {
  return m_error;
}

namespace internal {
inline SensorError process_error(const std::string &file, int line,
                                 const std::string &code,
                                 const SensorError &error, bool enable_log,
                                 bool enable_raise) {
  if (!error) return CEPTON_SUCCESS;
  SensorErrorWrapper wrapper(create_context_message(file, line, code));
  wrapper = error;
  if (enable_log || enable_raise)
    std::fprintf(stderr, "%s\n", wrapper.error().what());
  if (enable_raise && error.is_error()) std::terminate();
  return wrapper;
}
}  // namespace internal

}  // namespace cepton_sdk
