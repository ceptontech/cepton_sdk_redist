#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "cepton_sdk.h"
#pragma warning(disable:4996)

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
int frames_to_read = 10;
int lines = 0;
int lines_to_read = 40;

void print_stats(size_t n_points, struct CeptonSensorPoint const *p_points) {
	float  minx, miny, minz, maxx, maxy, maxz;
	float depth_avg = 0;

	if (n_points < 1) {
		printf("No points.");
		return;
	}

	minx = miny = minz = INFINITY;
	maxx = maxy = maxz = -INFINITY;

	for (size_t i = 0; i < n_points; i++) {
		struct CeptonSensorPoint const *p = &p_points[i];

		minx = (p->x < minx) ? p->x : minx;
		maxx = (p->x > maxx) ? p->x : maxx;
		miny = (p->y < miny) ? p->y : miny;
		maxy = (p->y > maxy) ? p->y : maxy;
		minz = (p->z < minz) ? p->z : minz;
		maxz = (p->z > maxz) ? p->z : maxz;
		depth_avg += p->z;
	} 
	depth_avg /= n_points;
	printf("stats: points: %d (x from %lf to %lf, y from %lf to %lf, z from %lf to %lf)\navg_depth: %f\n", 
    (int)n_points, minx, maxx, miny, maxy, minz, maxz, depth_avg);
}

void on_frame(int error_code, CeptonSensorHandle sensor, size_t n_points,
  struct CeptonSensorPoint const *p_points)
{
	struct CeptonSensorInformation const *pinfo;

  if (error_code < 0)
    return; // Handle error here

  if (frames >= frames_to_read) 
	  return;

  pinfo = cepton_sdk_get_sensor_information(sensor);

  printf("Sensor serial: %d Frame: %d\n", (int)pinfo->serial_number, frames+1);
  print_stats(n_points, p_points);
  frames++;
}


void on_scanline(int error_code, CeptonSensorHandle sensor, size_t n_points,
	struct CeptonSensorPoint const *p_points)
{
	struct CeptonSensorInformation const *pinfo;

	if (error_code < 0)
		return; // Handle error here
	if (lines >= lines_to_read) 
		return;

	pinfo = cepton_sdk_get_sensor_information(sensor);

	printf("Sensor serial: %d Scanlines: %d-%d\n", (int)pinfo->serial_number, lines+1, lines+2);
	print_stats(n_points, p_points);
	lines += 2;   // Each trigger of this callback gives us two scanlines
}

void on_event(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int event)
{
  if (error_code < 0)
    return; // Handle error here

  switch(event) {
    case CEPTON_EVENT_ATTACH:
      printf("Sensor: (serial number: %d, model: %s, firmware: %s) Attached\n",
        (int)p_info->serial_number, p_info->model_name, p_info->firmware_version);
	  // Do stuff for newly detected sensor
      break;
    case CEPTON_EVENT_FRAME:
      printf("New Frame\n");
      break;
    default:
      break;
  }

}

int main(int argc, char ** argv) {
  // Setup to listen for sensor output
  int error_code = cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_initialize failed: ");
	  return -1;
  }

  printf("Cepton SDK functions:\n");
  printf("number of sensors: %d\n", cepton_sdk_get_number_of_sensors());

  // Immediately setup to listen for frame data
  printf("Listen for %d frames...\n", frames_to_read);
  error_code = cepton_sdk_listen_frames(on_frame);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_listen_frames failed: ");
	  return -1;
  }

  while (frames < frames_to_read) {
	 // wait here for frames_to_read frames...
    common_sleep(frames_to_read*20);
  }
  error_code = cepton_sdk_unlisten_frames(on_frame);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_unlisten_frames failed: ");
	  return -1;
  }

  printf("\n\nListen for %d scanlines...\n", lines_to_read);
  error_code = cepton_sdk_listen_scanlines(on_scanline);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_listen_scanlines failed: ");
	  return -1;
  }

  while (lines < lines_to_read) {
    // wait here for lines_to_read lines...
    common_sleep(lines_to_read);
  }
  error_code = cepton_sdk_unlisten_scanlines(on_scanline);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_unlisten_lines failed: ");
	  return -1;
  }


  // Teardown connections etc.
  cepton_sdk_deinitialize();
  printf("Basic interaction sample completed successfully, press ENTER to quit.\n");
  getchar();
  return 0;
}
