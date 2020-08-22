#include "exporter.h"

void CeptonExporter::WriteFrame(SensorData &sd,
                                CeptonSensorImagePoint const *points,
                                size_t size) {
  got_frame = true;
  CsvExporter exp;
  exp.SetSplitTimestamp(split_timestamp);
  fs::path p = fs::absolute(base_dir);
  p.append(to_string(sd.serial_number));
  if (!fs::create_directories(p)) {
    ReportError("Failed to create directory " + p.string(), "write file");
  }
  bool append = single_file && file_ts != 0;
  if (!append) {
    if (include_invalid)
      file_ts = points[0].timestamp;
    else {
      size_t i;
      for (i = 0; i < size && !points[i].valid; i++)
        ;
      if (i == size) return;  // Nothing to write
      file_ts = points[i].timestamp;
    }
  }

  p.append(to_string(file_ts) + ".csv");
  if (!single_file)
    cout << "Writing Frame: (" << sd.frames_recorded + 1 << "/" << frame_count
         << ") " << p.string() << endl;
  else if (!append)
    cout << "Writing single file: " << p.string() << endl;
  exp.OpenFile(p, append);
  exp.Export(points, size, include_invalid, append);
  exp.CloseFile();

  sd.frames_recorded++;
  if (!capture_file.empty() && cepton_sdk_capture_replay_is_end()) {
    waiter.notify_one();
  }
  if (!streaming && sd.frames_recorded >= frame_count) {
    unique_lock<mutex> lock(waiter_mutex);
    completed_sensors++;
    if (sensor.size() == completed_sensors) {
      waiter.notify_one();
    }
  }
}

string GetOptionsHelp() {
  return (string) "Cepton Exporter Tool (version " + to_string(VERSION_MAJOR) +
         '.' + to_string(VERSION_MINOR) + '.' + to_string(VERSION_BUILD) + ")";
}

CeptonExporter::CeptonExporter()
    : options("cepton_exporter", GetOptionsHelp()) {
  options.add_options()
      // clang-format off
    ("s,serial-number", "Target sensor serial number, dump all connected sensors if unspecified", cxxopts::value<uint64_t>(), "")
    // ("format", "Output file format (only CSV for now)", cxxopts::value<string>(), "")
    ("b,base-dir", "Base folder to store frame data", cxxopts::value<string>(), "")
    ("n,frames", "Number of frames to dump, default 5", cxxopts::value<unsigned>(), "")
    ("c,capture", "Specify pcap file to read from", cxxopts::value<string>(), "")
    ("all-points", "Export all points, including the invalid ones")
    ("single-file", "Export points to a single file")
    ("split-timestamp", "Split timestamp into seconds and microsecond parts")
    ("h,help", "Print Help")
      // clang-format on
      ;

  // options.add_options("internal")
  //   // clang-format off
  //   // clang-format on
  //   ;
}

CeptonExporter::~CeptonExporter() {
  if (sdk_initialized) {
    cepton_sdk_deinitialize();
    sdk_initialized = false;
  }
}

void CeptonExporter::ReportError(CeptonSensorErrorCode err, string when) {
  string err_str = cepton_get_error_code_name(err);
  ReportError(err_str, when);
}

void CeptonExporter::ReportError(string err_str, string when) {
  if (when.empty())
    cout << "ERROR: " << err_str << endl;
  else
    cout << "ERROR: (" << when << ") " << err_str << endl;
}

// struct SensorImageData {
//  CeptonSensorHandle handle;
//  CeptonSensorImagePoint *c_points;
//  size_t n_points;
//};

void CeptonExporter::CbCeptonSensorImageData(
    CeptonSensorHandle handle, size_t n_points,
    const struct CeptonSensorImagePoint *c_points, void *user_data) {
  reinterpret_cast<CeptonExporter *>(user_data)->FrameData(handle, n_points,
                                                           c_points);
}

void CeptonExporter::FrameData(CeptonSensorHandle handle, size_t n_points,
                               const struct CeptonSensorImagePoint *c_points) {
  if (sensor.count(handle)) {
    auto &sd = sensor[handle];
    if (streaming || sd.frames_recorded < frame_count) {
      WriteFrame(sd, c_points, n_points);
    } else {
      // Do nothing
    }
  } else {
    CeptonSensorInformation info;
    auto err = cepton_sdk_get_sensor_information(handle, &info);
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "frame data callback");
      return;
    }
    if (target_sn && target_sn != info.serial_number) return;
    cout << "Detected sensor #" << to_string(info.serial_number) << endl;
    SensorData sd = {info.serial_number, 0, 0};
    sensor.emplace(handle, sd);
  }
}

void CeptonExporter::CbCeptonSensorError(CeptonSensorHandle handle,
                                         CeptonSensorErrorCode error_code,
                                         const char *error_msg,
                                         const void *error_data,
                                         size_t error_data_size,
                                         void *user_data) {
  ReportError(error_msg, "Sensor");
}

int CeptonExporter::InitializeSDK() {
  CeptonSensorErrorCode err;
  CeptonSDKOptions opts = cepton_sdk_create_options();
  if (streaming) {
    opts.frame.mode = CEPTON_SDK_FRAME_STREAMING;
  } else {
    // Use cover mode as the default mode
    opts.frame.mode = CEPTON_SDK_FRAME_COVER;
  }
  
  if (!capture_file.empty())
    opts.control_flags |= CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  err = cepton_sdk_initialize(CEPTON_SDK_VERSION, &opts, CbCeptonSensorError,
                              this);
  if (err != CEPTON_SUCCESS) {
    ReportError(err, "initialize");
    return -1;
  }

  if (!capture_file.empty()) {
    // attempt to discover the sensors in the pcap
    err = cepton_sdk_capture_replay_open(capture_file.c_str());
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "load capture");
      return -1;
    }
    err = cepton_sdk_capture_replay_set_speed(0);
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "set replay speed");
      return -1;
    }
    err = cepton_sdk_capture_replay_resume_blocking(1.2f);
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "pre-replay");
      return -1;
    }
    err = cepton_sdk_capture_replay_seek(0);
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "rewind");
      return -1;
    }
    err = cepton_sdk_clear();
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "clear");
      return -1;
    }
  }

  err = cepton_sdk_listen_image_frames(CbCeptonSensorImageData, this);
  if (err != CEPTON_SUCCESS) {
    ReportError(err, "listen frames");
    return -1;
  }

  if (!capture_file.empty()) {
    err = cepton_sdk_capture_replay_resume();
    if (err != CEPTON_SUCCESS) {
      ReportError(err, "load capture");
      return -1;
    }
  }

  return 0;
}

int CeptonExporter::CleanupSDK() {
  if (!capture_file.empty()) cepton_sdk_capture_replay_close();
  cepton_sdk_unlisten_image_frames();
  cepton_sdk_deinitialize();
  return 0;
}

void PrintHelp(cxxopts::Options &options) {
  cout << options.help({"", "Group"});
  cout << R"NOTES(Notes:
  By default, points are stored in <base-dir>/<serial-number>/<timestamp>.csv
  where timestamp is the microsecond value of linux epoch time. Only full
  frames are dumped. In --single-file mode, points are stored in
  <base-dir>/<serial-number>/<timestamp>.csv
)NOTES" << endl;
  cout << R"HELP(Examples:
  # Dump all sensors 10 frames each to c:/tmp/<serial_number>/<timestamp>.csv
  cepton_exporter -n 10 --base-dir=c:/tmp

  # Dump 5 frames sensor #8883 to ./8883/<timestamp>.csv
  cepton_exporter -s 8883
)HELP" << endl;
}

int CeptonExporter::ParseOptions(int argc, char **argv) {
  if (argc == 1) {
    PrintHelp(options);
    return 0;
  }

  try {
    int err = 0;
    int action = 0;
    cxxopts::ParseResult result = options.parse(argc, argv);

    for (int i = 1; i < argc; i++) {
      ReportError("Extra argument ignored: " + string(argv[i]));
    }

    if (result.count("help")) {
      PrintHelp(options);
      return 0;
    }

    if (result.count("serial_number")) {
      target_sn = result["serial_number"].as<uint64_t>();
    } else {
      target_sn = 0;
    }

    if (result.count("base-dir")) {
      base_dir = result["base-dir"].as<string>();
    } else {
      base_dir = ".";
    }

    if (result.count("frames")) {
      frame_count = result["frames"].as<unsigned>();
    } else {
      frame_count = 5;
    }

    if (result.count("capture")) {
      capture_file = fs::absolute(result["capture"].as<string>());
      if (!fs::exists(capture_file)) {
        ReportError("Capture file doesn't exist " + capture_file.string());
        return -1;
      }
    }

    if (result.count("single-file")) {
      if (capture_file.empty()) {
        ReportError("--single-file can only be used when loading captures.");
        return -1;
      }
      if (!result.count("frames")) {
        streaming = true;
      }
      single_file = true;
    }

    if (result.count("split-timestamp")) {
      split_timestamp = true;
    }

    include_invalid = result.count("all-points") > 0;

    if (InitializeSDK() == 0) {
      unique_lock<mutex> lock(waiter_mutex);
      waiter.wait_for(lock, chrono::milliseconds(1500));
      if (sensor.size() == 0) {
        ReportError("No sensor found");
        return -1;
      }

      int timeouts = 0;
      while (completed_sensors < sensor.size()) {
        // We do a constant 0.5 second wait, regardless of when the condition
        // variable receives notify_one/notify_all. This because when running on
        // windows, the waiter would occasionaly be triggered early, resulting
        // in the program not running to completion on all frames
        waiter.wait_for(lock, chrono::milliseconds(500));
        if (!capture_file.empty()) {
          if (cepton_sdk_capture_replay_is_end()) break;
        }
        if (!got_frame) {
          timeouts++;
        }
        got_frame = false;
        if (timeouts > 4) {
          ReportError("Timed out waiting for " + to_string(frame_count) +
                      " frames");
          break;
        }
      }
    }
    CleanupSDK();
  } catch (const cxxopts::OptionException &e) {
    ReportError(CEPTON_SUCCESS, e.what());
    return -1;
  }
  return 0;
}

int main(int argc, char **argv) {
  CeptonExporter ex;
  return ex.ParseOptions(argc, argv);
}
