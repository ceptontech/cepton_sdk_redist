#include <string.h>
#include <math.h>

#include <dw/sensors/plugins/lidar/LidarDecoder.h>
#include "cepton_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_initialize(const float32_t spinFrequency);
__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_release();

__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_getConstants(_dwLidarDecoder_constants *constants);

__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_decodePacket(dwLidarDecodedPacket *output,
                                      const uint8_t *buffer,
                                      const size_t length);
__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_synchronize(const uint8_t *buffer,
                                     const size_t length,
                                     size_t *remaining);
__attribute__((visibility("default"))) dwStatus _dwLidarDecoder_validatePacket(const uint8_t *buffer,
                                    const size_t length);


#ifdef __cplusplus
}
#endif

