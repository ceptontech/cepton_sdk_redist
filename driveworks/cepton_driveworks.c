#include "cepton_driveworks.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

// #define LOG printf
#define LOG (void)

const CeptonSensorHandle DRIVE_WORKS_HANDLE = 0x181212; // Any arbitrary number
#define MAX_POINT_BUFFERED 200
struct CeptonSensorImagePoint point_buffer[MAX_POINT_BUFFERED]; // Max
size_t buffer_size = 0;

void cepton_image_callback(CeptonSensorHandle handle, size_t n_points,
  const struct CeptonSensorImagePoint *c_points, void *user_data) {
  size_t to_copy = n_points;
  if (to_copy + buffer_size > MAX_POINT_BUFFERED) {
    to_copy = MAX_POINT_BUFFERED - buffer_size;
  }
  if (to_copy > 0) {
    memcpy(&point_buffer[buffer_size], c_points, sizeof(struct CeptonSensorImagePoint)*to_copy);
    buffer_size += to_copy;
  }
}

static inline float square(float x) { return x*x; }

// HACK: decodPacket pass in 0 length, have to borrow from the last call
static size_t last_validate_length = 0;

dwStatus _dwLidarDecoder_initialize(const float32_t spinFrequency) {
    LOG("_dwLidarDecoder_initialize(%f)\n", spinFrequency);
  int err;
  struct CeptonSDKOptions opt = cepton_sdk_create_options();
  opt.control_flags = CEPTON_SDK_CONTROL_DISABLE_NETWORK;
  err = cepton_sdk_initialize(CEPTON_SDK_VERSION, &opt, NULL, NULL);
  if (err == CEPTON_SUCCESS) {
    cepton_sdk_listen_image_frames(cepton_image_callback, NULL);
  }
  return err == CEPTON_SUCCESS ? DW_SUCCESS : DW_FAILURE;
}

dwStatus _dwLidarDecoder_release() {
  LOG("_dwLidarDecoder_release()\n");
  cepton_sdk_deinitialize();
  return DW_SUCCESS;
}

dwStatus _dwLidarDecoder_getConstants(_dwLidarDecoder_constants *constants) {
  LOG("_dwLidarDecoder_getConstants()\n");
  strncpy(constants->properties.deviceString, "Cepton Lidar",
    sizeof(constants->properties.deviceString));

  constants->properties.spinFrequency  = 10.0;

  constants->properties.packetsPerSecond = 2170;
  constants->properties.packetsPerSpin = 217;
  
  constants->properties.pointsPerSecond = 312500; // 3.2us point interval
  constants->properties.pointsPerPacket = 144;
  constants->properties.pointsPerSpin = 31250;
  constants->properties.pointStride = sizeof(dwLidarPointXYZI);

  constants->headerSize = 16;
  constants->maxPayloadSize = 1540;
  return DW_SUCCESS;
}

dwStatus _dwLidarDecoder_decodePacket(dwLidarDecodedPacket *output,
  const uint8_t *buffer,
  size_t length) {

  LOG("_dwLidarDecoder_decodePacket(%c%c%c%c, %ld)\n", buffer[0], buffer[1], buffer[2], buffer[3], length);
  
  if (length == 0)
    length = last_validate_length;

  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t ts = tv.tv_sec * 1000000 + tv.tv_usec;

  cepton_sdk_mock_network_receive(DRIVE_WORKS_HANDLE, ts, buffer, length);

  if (output->maxPoints && buffer_size > output->maxPoints) {
    LOG("Buffer too big %d > %d. This should never happen\n", (int)buffer_size, output->maxPoints);
    buffer_size = output->maxPoints;
  }

  if (buffer_size > 0) {
    LOG("Got %ld points\n", buffer_size);
    struct CeptonSensorImagePoint *l = &point_buffer[buffer_size-1];    
    // Copy buffer over
    output->hostTimestamp = point_buffer[0].timestamp;
    output->sensorTimestamp = point_buffer[0].timestamp;
    output->duration = (l->timestamp - output->hostTimestamp);
    output->nPoints = buffer_size;
    dwLidarPointXYZI *pOut = (dwLidarPointXYZI *)output->pointsXYZI;
    // dwLidarPointRTHI *pOutSpherical = (dwLidarPointRTHI *)output->pointsRTHI;
    LOG("%p\n", pOut);

    for (size_t i = 0; i< buffer_size; i++) {
      struct CeptonSensorImagePoint *p = &point_buffer[i];

      float hypotenuse_small = sqrt(square(p->image_x) + square(p->image_z) + 1.0f);
      float ratio = p->distance / hypotenuse_small;
      pOut[i].x = -p->image_x * ratio;
      pOut[i].y = ratio;
      pOut[i].z = -p->image_z * ratio;
      pOut[i].intensity = p->intensity;

      // pOutSpherical[i].phi = atan(pOut[i].y/pOut[i].z);
      // pOutSpherical[i].radius = sqrt(pOut[i].x*pOut[i].x + pOut[i].y*pOut[i].y + pOut[i].z*pOut[i].z);
      // pOutSpherical[i].theta = acos(pOut[i].z/pOutSpherical[i].radius);
      // pOutSpherical[i].intensity = p->intensity;
    }
    buffer_size = 0;
  }
  else {
    output->hostTimestamp = ts;
    output->sensorTimestamp = ts;
    output->duration = 0;
    output->nPoints = 0;
  }
  return DW_SUCCESS;
}

dwStatus _dwLidarDecoder_synchronize(const uint8_t *buffer, const size_t length, size_t *remaining) {
  LOG("_dwLidarDecoder_synchronize(length=%d)\n", (int)length);
  *remaining = 0;
  return DW_SUCCESS;
}

dwStatus _dwLidarDecoder_validatePacket(const uint8_t *buffer, const size_t length) {
  last_validate_length = length;
  LOG("_dwLidarDecoder_validatePacket(%c%c%c%c, length=%d)\n", buffer[0], buffer[1], buffer[2], buffer[3], (int)length);
  return DW_SUCCESS;
}

