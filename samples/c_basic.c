/**
 * Sample code for general C SDK usage.
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <cepton_sdk.h>

void check_sdk_error() {
  const char *error_msg;
  int error_code = cepton_sdk_get_error(&error_msg);
  if (error_code != CEPTON_SUCCESS) {
    printf("%s: %s\n", cepton_get_error_code_name(error_code), error_msg);
    exit(1);
  }
}

void cepton_sdk_error_handler(CeptonSensorHandle handle,
                              CeptonSensorErrorCode error_code,
                              const char *error_msg, const void *error_data,
                              size_t error_data_size, void *user_data) {
  printf("Got error: %s\n", error_msg);
}

int n_frames = 0;

void image_frame_callback(CeptonSensorHandle handle, size_t n_points,
                          const struct CeptonSensorImagePoint *c_points,
                          void *user_data) {
  ++n_frames;
  printf("Got %d frames\n", n_frames);
}

int main() {
  // Initialize
  struct CeptonSDKOptions options = cepton_sdk_create_options();
  options.frame.mode = CEPTON_SDK_FRAME_COVER;
  cepton_sdk_initialize(CEPTON_SDK_VERSION, &options, cepton_sdk_error_handler, NULL);
  check_sdk_error();

  // Wait for sensor
  int n_sensors;
  printf("Waiting for sensors to connect\n");
  while (!(n_sensors = (int)cepton_sdk_get_n_sensors()))
    ;
  for (int i = 0; i < n_sensors; ++i) {
    struct CeptonSensorInformation sensor_info;
    cepton_sdk_get_sensor_information_by_index(0, &sensor_info);
    check_sdk_error();
    printf("Sensor: %i\n", (int)sensor_info.serial_number);
  }

  // Listen for frames
  cepton_sdk_listen_image_frames(image_frame_callback, NULL);
  check_sdk_error();

  // Sleep
  while (n_frames < 10)
    ;

  // Deinitialize
  cepton_sdk_deinitialize();
  check_sdk_error();
  return 0;
}