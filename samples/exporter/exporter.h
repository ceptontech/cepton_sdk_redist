#pragma once

#define NOMINMAX

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <condition_variable>

#if defined(__cplusplus) && __cplusplus >= 201703L && defined(__has_include)
#if __has_include(<filesystem>)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs {
using namespace std::filesystem;
using ifstream = std::ifstream;
using ofstream = std::ofstream;
using fstream = std::fstream;
}  // namespace fs
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs {
using namespace ghc::filesystem;
using ifstream = ghc::filesystem::ifstream;
using ofstream = ghc::filesystem::ofstream;
using fstream = ghc::filesystem::fstream;
}  // namespace fs
#endif

#include "cepton_sdk.h"
#include "cxxopts.hpp"

using namespace std;

class CeptonExporter {
 public:
  CeptonExporter();
  ~CeptonExporter();

  int InitializeSDK();
  int CleanupSDK();
  int ParseOptions(int argc, char **argv);

 private:
  static void ReportError(CeptonSensorErrorCode err, string when);
  static void ReportError(string err, string when = "");

  static void CbCeptonSensorError(CeptonSensorHandle handle,
                                  CeptonSensorErrorCode error_code,
                                  const char *error_msg, const void *error_data,
                                  size_t error_data_size, void *user_data);


  cxxopts::Options options;
  bool sdk_initialized = false;

  string base_dir = ".";
  fs::path capture_file;
  bool streaming = false;
  unsigned frame_count = 5;
  uint64_t target_sn = 0;
  bool include_invalid = false;
  bool single_file = false;
  bool split_timestamp = false;
  int64_t file_ts = 0;

  unsigned completed_sensors = 0;
  bool got_frame = false;

  struct SensorData {
    uint64_t serial_number;
    unsigned frames_recorded;
    uint64_t last_ts;
  };

  map<CeptonSensorHandle, SensorData> sensor;
  mutable mutex waiter_mutex;
  mutable condition_variable waiter;

  void FrameData(CeptonSensorHandle handle, size_t n_points,
                 const struct CeptonSensorImagePoint *c_points);
  void WriteFrame(SensorData &sd, CeptonSensorImagePoint const *points,
                  size_t size);

  static void CbCeptonSensorImageData(
      CeptonSensorHandle handle, size_t n_points,
      const struct CeptonSensorImagePoint *c_points, void *user_data);
};

class CsvExporter {
 private:
  shared_ptr<ostream> export_stream;
  bool split_timestamp = false;

  void ExportPoint(CeptonSensorImagePoint const &p);

 public:
  int Export(CeptonSensorImagePoint const *points, size_t size,
             bool include_invalid, bool append);

  // Subclass interfaces (with default implementations)
  int OpenFile(fs::path &file_name, bool append = false) {
    auto file_stream = make_shared<fs::ofstream>();
    auto openmode = ios_base::out | ios_base::binary | ios_base::trunc;
    if (append) openmode = ios_base::out | ios_base::binary | ios_base::app;
    file_stream->open(file_name, openmode);
    if (!file_stream->bad()) export_stream = file_stream;
    return file_stream->bad() ? -1 : 0;
  }

  void SetSplitTimestamp(bool st) { split_timestamp = st; }

  int CloseFile() {
    if (export_stream) export_stream->flush();
    export_stream.reset();
    return 0;
  }
};
