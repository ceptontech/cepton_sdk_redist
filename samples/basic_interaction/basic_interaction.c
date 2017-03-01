// #include <vld.h> // https://vld.codeplex.com/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "cepton_sdk.h"
#pragma warning(disable:4996)

int frames = 0;
int lines = 0;

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
		//printf("Got point %f, %f, %f, %f\n", p->x, p->y, p->z, p->intensity);

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
  frames++;
  pinfo = cepton_sdk_get_sensor_information(sensor);

  printf("Sensor serial: %d Frame: %d\n", (int)pinfo->serial_number, frames);
  print_stats(n_points, p_points);
}


void on_scanline(int error_code, CeptonSensorHandle sensor, size_t n_points,
	struct CeptonSensorPoint const *p_points)
{
	struct CeptonSensorInformation const *pinfo;

	if (error_code < 0)
		return; // Handle error here

	lines++;
	pinfo = cepton_sdk_get_sensor_information(sensor);

	printf("Sensor serial: %d Scanline: %d\n", (int)pinfo->serial_number, lines);
	print_stats(n_points, p_points);
}

void on_event(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int event)
{
  if (error_code < 0)
    return; // Handle error here

  switch(event) {
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

int main(int argc, char ** argv) {
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
  }
  error_code = cepton_sdk_unlisten_frames(on_frame);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_unlisten_frames failed: ");
	  return 0;
  }

  error_code = cepton_sdk_listen_scanlines(on_scanline);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_listen_scanlines failed: ");
	  return 0;
  }

  while (lines < 20) {
	  // wait here for 20 lines...
  }
  error_code = cepton_sdk_unlisten_scanlines(on_scanline);
  if (error_code != CEPTON_SUCCESS) {
	  perror("cepton_sdk_unlisten_lines failed: ");
	  return 0;
  }


  // Teardown connections etc.
  cepton_sdk_deinitialize();

  return 1;
}
