// #include <vld.h> // https://vld.codeplex.com/
#include <stdio.h>
#include <stdlib.h>

#include "cepton_sdk.h"
#pragma warning(disable:4996)

struct CeptonSensorInformation const *pInfo = NULL;

void on_receive(int error_code, CeptonSensorHandle sensor, size_t n_points,
  struct CeptonSensorPoint const *p_points)
{
  if (error_code < 0)
    return; // Handle error here

  for (size_t i = 0; i < n_points; i++) {
    struct CeptonSensorPoint const *p = &p_points[i];
    printf("Got point %f, %f, %f, %f\n", p->x, p->y, p->z, p->intensity);
  }
  // Do stuff with the data points
}

void on_event(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int event)
{
  if (error_code < 0)
    return; // Handle error here

  switch(event) {
    case CEPTON_EVENT_ATTACH:
      printf("Attached\n");
      break;
    case CEPTON_EVENT_FRAME:
      printf("New Frame\n");
      break;
    default:
      break;
  }
  // Do stuff for newly detected sensor

  pInfo = p_info;
}

int main(int argc, char ** argv) {
  // Setup to listen for sensor output
  int error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event);
  if (error_code < 0)
    return 0; // Handle error here

  // Immediately setup to listen for frame data
  error_code = cepton_sdk_listen_frames(on_receive);
  if (error_code < 0)
    return 0; // Handle error here

  // Do other work or just wait until done with sensor
  printf("Press ENTER to continue. \n");
  getchar();

  // Teardown connections etc.
  cepton_sdk_deinitialize();

  return 1;
}
