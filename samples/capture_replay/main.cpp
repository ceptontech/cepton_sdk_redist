#include <cepton_sdk.h>
#include <string>
#include "capture_replay.hpp"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

void common_sleep(int milliseconds) {
  //sleep:
#ifdef _WIN32
  Sleep(milliseconds);
#else
  usleep(milliseconds * 1000);
#endif
}


int frames = 0;

void on_frame(int error_code, CeptonSensorHandle sensor, size_t n_points,
  struct CeptonSensorPoint const *p_points)
{
  struct CeptonSensorInformation const *pinfo;

  if (error_code < 0)
    return; // Handle error here
  frames++;
  pinfo = cepton_sdk_get_sensor_information(sensor);

  printf("Sensor serial: %d Frame: %d\n", (int)pinfo->serial_number, frames);
}

void on_event(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int event)
{
  if (error_code < 0)
    return; // Handle error here

  switch (event) {
  case CEPTON_EVENT_ATTACH:
    printf("Sensor: (serial number: %d model: %s) Attached\n", (int)p_info->serial_number, p_info->model_name);
    // Do stuff for newly detected sensor
    break;
  case CEPTON_EVENT_FRAME:
    printf("New Frame\n");
    break;
  default:
    break;
  }
}

int basic_interaction() {
  // Setup to listen for sensor output
  int error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event);
  if (error_code != CEPTON_SUCCESS) {
    perror("cepton_sdk_initialize failed: ");
    return 0;
  }

  printf("Cepton SDK functions:\n");
  printf("number of sensors: %d\n", cepton_sdk_get_number_of_sensors());

  // Immediately setup to listen for frame data
  error_code = cepton_sdk_listen_frames(on_frame);
  if (error_code != CEPTON_SUCCESS) {
    perror("cepton_sdk_listen_frames failed: ");
    return 0;
  }

  while (frames < 10) {
    // wait here for 10 frames...
    common_sleep(200);
  }
  error_code = cepton_sdk_unlisten_frames(on_frame);
  if (error_code != CEPTON_SUCCESS) {
    perror("cepton_sdk_unlisten_frames failed: ");
    return 0;
  }

  // Teardown connections etc.
  cepton_sdk_deinitialize();
  return 1;
}

int main(int argc, char **argv) {
  if (argc <= 1) {
    printf("Usage: capture_replay <pcapfile>\n");
    return 0;
  }

  cepton::CaptureReplay replay;

  if (!replay.load_capture(argv[1])) {
    printf("Failed to open pcap file\n");
    return 0;
  }

  // If we get here, the replay is already going
  // Code from here can be 100% same as if dealing with real sensors
  basic_interaction();

}