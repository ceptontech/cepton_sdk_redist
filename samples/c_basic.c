/**
 * Sample code for general C sdk usage.
 */
#include <stdio.h>
#include <time.h>

#include "cepton_sdk.h"

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
  printf("Start\n");
  fflush(stdout);
  struct CeptonSDKOptions options = cepton_sdk_create_options();
  options.frame.mode = CEPTON_SDK_FRAME_TIMED;
  options.frame.length = 0.1f;
  CeptonSensorErrorCode ret;
  ret = cepton_sdk_initialize(CEPTON_SDK_VERSION, &options, NULL, NULL);
  if (ret != CEPTON_SUCCESS) printf("%s\n", cepton_get_error_code_name(ret));
  ret = cepton_sdk_listen_image_frames(image_frame_callback, NULL);
  if (ret != CEPTON_SUCCESS) printf("%s\n", cepton_get_error_code_name(ret));

  while (frames_got < 100)
    ;  // Just spin loop for lack of cross platform sleep in C

  cepton_sdk_deinitialize();

  return 0;
}