#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <chrono>
#include <thread>

#ifdef _WIN32
#include <Windows.h>
#include "getopt_windows.h"
#else
#include <getopt.h>
#include <unistd.h>
#endif

#include "cepton_sdk_api.hpp"
#include "cepton_sdk_util.hpp"

static struct option long_options[] = {
    {"capture_file", required_argument, nullptr, 'c'},
    {"serial_number", required_argument, nullptr, 's'},
    {"frames", required_argument, nullptr, 'n'},
    {"split", required_argument, nullptr, 'l'},
    {"format", required_argument, nullptr, 'f'},
    {"convert", no_argument, nullptr, 'x'},
    {"help", no_argument, nullptr, 'h'},
};
static const char *short_options = "c:s:n:l:f:xh";

// Command line parameters
const char *replay_file = "";
const char *fname = nullptr;
int serial_number = 0;
enum {
  FILE_MODE_CSV = 0,
  FILE_MODE_BIN = 1,
  FILE_MODE_JSON = 2,
};
int file_mode = FILE_MODE_CSV;

bool convert = false;
size_t required_frames = 10;
size_t split_frame = 0;

// Internal states
bool file_opened = false;  // Only useful for closing the bracket of JSON files.
size_t total_frames = 0;
CeptonSensorHandle filter_by_handle = CEPTON_NULL_HANDLE;

char current_file_name[256];

void open_file() {
  if (file_opened) return;

  if (split_frame > 0) {
    size_t nf = (total_frames + 1) / split_frame;  // Where next point will land
    snprintf(current_file_name, sizeof(current_file_name), "%s%d", fname,
             (int)nf);
  } else {
    strncpy(current_file_name, fname, sizeof(current_file_name));
  }
  // printf("Opening file: %s\n", current_file_name);
  FILE *fh = fopen(current_file_name, "w+b");  // This will truncate the file
  if (file_mode == FILE_MODE_JSON) {
    fprintf(fh, "[\n");
  }
  fclose(fh);
  file_opened = true;
}

void close_file() {
  if (!file_opened) return;

  if (file_mode == FILE_MODE_JSON) {
    FILE *fh = fopen(current_file_name, "r+b");
    fseek(fh, -2, SEEK_END);  // Reverse the last ",\n"
    fwrite("\n]\n", 3, 1, fh);
    fclose(fh);
  }
  file_opened = false;
}

void write_points(struct CeptonSensorImagePoint const *points,
                  size_t n_points) {
  // printf("Writing %d points to file\n", (int)n_points);
  FILE *fh = fopen(current_file_name, "ab");
  switch (file_mode) {
    case FILE_MODE_BIN:
      fwrite(points, sizeof(CeptonSensorImagePoint), n_points, fh);
      break;
    case FILE_MODE_CSV:
      for (size_t i = 0; i < n_points; i++) {
        CeptonSensorImagePoint const &p = points[i];
        if (convert) {
          float wx, wy, wz;  // For converting
          cepton_sdk::util::convert_image_point_to_point(
              p.image_x, p.image_z, p.distance, wx, wy, wz);
          fprintf(fh, "%.6f,%f,%f,%f,%f,%d,%d\n", (double)p.timestamp / 1e6, wx,
                  wy, wz, p.intensity, (int)p.return_number, (int)p.valid);
        } else {
          fprintf(fh, "%.6f,%f,%f,%f,%f,%d,%d\n", (double)p.timestamp / 1e6,
                  p.image_x, p.distance, p.image_z, p.intensity,
                  (int)p.return_number, (int)p.valid);
        }
      }
      break;
    case FILE_MODE_JSON:
      for (size_t i = 0; i < n_points; i++) {
        CeptonSensorImagePoint const &p = points[i];
        if (convert) {
          float wx, wy, wz;  // For converting
          cepton_sdk::util::convert_image_point_to_point(
              p.image_x, p.image_z, p.distance, wx, wy, wz);
          fprintf(fh,
                  "{\"timestamp\":%.6f,\"x\":%f,\"y\":%f,\"z\":%f,"
                  "\"intensity\":%f,\"return_number\":%d,\"valid\":%d},\n",
                  (double)p.timestamp / 1e6, wx, wy, wz, p.intensity,
                  (int)p.return_number, (int)p.valid);
        } else {
          fprintf(fh,
                  "{\"timestamp\":%.6f,\"image_x\":%f,\"distance\":%f,\"image_"
                  "z\":%f,"
                  "\"intensity\":%f,\"return_number\":%d,\"valid\":%d},\n",
                  (double)p.timestamp / 1e6, p.image_x, p.distance, p.image_z,
                  p.intensity, (int)p.return_number, (int)p.valid);
        }
      }
  }
  fclose(fh);
}

void on_image_frame(CeptonSensorHandle sensor, size_t n_points,
                    struct CeptonSensorImagePoint const *p_points,
                    void *user_data) {
  if (filter_by_handle == CEPTON_NULL_HANDLE) {
    if (serial_number > 0) {
      // When this call fails, handle is not updated and the points will be
      // skipped
      cepton_sdk_get_sensor_handle_by_serial_number(serial_number,
                                                    &filter_by_handle);
    } else {
      filter_by_handle =
          sensor;  // Will not take any other sensor's data anymore.
    }
  }

  if (filter_by_handle != sensor) return;

  open_file();
  write_points(p_points, n_points);
  total_frames++;
  if (total_frames >= required_frames)
    close_file();
  else if (split_frame && (total_frames % split_frame == 0))
    close_file();
}

static void print_help() {
  printf(R"(
Cepton data_exporter version 1.1

Usage: data_exporter <options> <outputfile>

Options are:
  -c, --capture_file=<capture_file>
      Load from capture file instead of real lidar. Capture files are usually
      pcap files generated by tcpdump.

  -s, --serial_number=<serial>
      Specify the serial number of the source lidar. Useful when multiple
      lidars are connected. If not specified, first discovered lidar will be
      used. data_exporter will never dump points from different lidars.

  -n, --frames=<count>
      Total number of frames to dump. Default is 10

  -l, --split=<split_count>
      Generate numbered file so that each file holds <split_count> frames. If
      not specified, all the frames will be dumped to a single file.

  -f, --format=<format>
      Valid formats are csv, json, or bin. Default is csv. Binary files uses
      CeptonSensorImagePoint structure for each point. CSV and JSON will 
      convert timestamps to seconds since epoch, while BIN will retain the 
      int64 value of microseconds since epoch.

  -x, --convert
     If specified, points are converted to world coordinates XYZI. By default
     points are generated in "image coordinate", see documentations for
     CeptonSensorImagePoint for more details.
)");
}

const int MAX_TOTAL_FRAMES = 10000;
const int MAX_FILES = 1000;

int main(int argc, char **argv) {
  if (argc == 1) {
    print_help();
    return 0;
  }

  int iarg = 0;
  while (iarg != -1) {
    int index;
    iarg = getopt_long(argc, argv, short_options, long_options, &index);
    switch (iarg) {
      case 'h':
        print_help();
        return 0;
      case 'c':
        replay_file = optarg;
        break;
      case 's':
        serial_number = atoi(optarg);
        break;
      case 'n':
        required_frames = atoi(optarg);
        break;
      case 'l':
        split_frame = atoi(optarg);
        break;
      case 'f':
        if (strcmp(optarg, "json") == 0)
          file_mode = FILE_MODE_JSON;
        else if (strcmp(optarg, "bin") == 0)
          file_mode = FILE_MODE_BIN;
        else if (strcmp(optarg, "csv") == 0)
          file_mode = FILE_MODE_CSV;
        else {
          printf("Unrecognized format\n");
          return 0;
        }
        break;
      case 'x':
        convert = true;
        break;
      case '?':
        return 0;  // getopt already printed error
    }
  }

  if (convert && file_mode == FILE_MODE_BIN) {
    printf("Cannot convert binary format\n");
    return -1;
  }

  if (required_frames > MAX_TOTAL_FRAMES) {
    printf("Frame count exceed maximum allowed %d\n", MAX_TOTAL_FRAMES);
    return -1;
  }

  if (split_frame > 0 && required_frames / split_frame > MAX_FILES) {
    printf("Will not generate more than %d files\n", MAX_FILES);
    return -1;
  }

  if (argc > optind + 1) {
    printf("Extra argument after filename ignored\n");
  } else if (argc < optind + 1) {
    printf("Expect a file name\n");
    return -1;
  }

  fname = argv[optind];
  if (fname[0] == '-') {
    printf("Invalid file name: %s\n", fname);
    return -1;
  }

  open_file();  // Make sure open file before start listening

  cepton_sdk::Options opts = cepton_sdk::create_options();
  opts.frame.mode = CEPTON_SDK_FRAME_COVER;  // Each cover is a frame
  cepton_sdk::api::check_error_code(
      cepton_sdk::api::initialize(opts, replay_file));

  cepton_sdk::api::check_error_code(
      cepton_sdk::listen_image_frames(on_image_frame, nullptr));
  if (replay_file) {
    cepton_sdk::api::check_error_code(
        cepton_sdk::capture_replay::set_enable_loop(true));
  }
  while (total_frames < required_frames) {
    cepton_sdk::api::check_error_code(cepton_sdk::api::wait());
  }
  cepton_sdk::api::check_error_code(cepton_sdk::deinitialize());

  close_file();

  return 0;
}
