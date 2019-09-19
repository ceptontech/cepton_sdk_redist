/**
 * Sample code for general C sdk usage.
 */
#include <stdio.h>
#include <time.h>

#include <cepton_sdk.h>

void check_sdk_error() {
  const char *error_msg;
  const auto error_code = cepton_sdk_get_error(&error_msg);
  printf("%s: %s\n", cepton_get_error_code_name(error_code), error_msg);
  exit(1);
}

int frames_got = 0;
time_t first_frame_time = 0;
time_t current_frame_time = 0;

void image_frame_callback(CeptonSensorHandle handle, size_t n_points,
                          const struct CeptonSensorImagePoint *c_points,
                          void *user_data) {
  time_t t = time(NULL);

  if (frames_got == 0) first_frame_time = t;
  frames_got++;
  if (frames_got < 50)
    printf("Frame: %4d Time: %lld\n", frames_got, (long long)t);
  else
    printf("Frame: %4d Time: %lld Frame rate: %.1fHz\n", frames_got,
           (long long)t, frames_got * 1.0 / (t - first_frame_time));
}

int main() {
  // Initialize
  struct CeptonSDKOptions options = cepton_sdk_create_options();
  options.frame.mode = CEPTON_SDK_FRAME_TIMED;
  options.frame.length = 0.1f;
  cepton_sdk_initialize(CEPTON_SDK_VERSION, &options, NULL, NULL);
  check_sdk_error();

  // Listen for frames
  cepton_sdk_listen_image_frames(image_frame_callback, NULL);
  check_sdk_error();

  // Run
  while (frames_got < 100)
    ;  // Just spin loop for lack of cross platform sleep in C

  // Deinitialize
  cepton_sdk_deinitialize();
  check_sdk_error();
}