#pragma once

#include <cmath>

#include <cepton_sdk.h>

inline square(float x) { return x * x; }

void convert_image_point_to_point(const CeptonSensorImagePoint &image_point,
                                  CeptonSensorPoint &point) {
  float hypotenuse_small =
      std::sqrt(square(image_point.image_x) + square(image_point.image_z) + 1);
  float ratio = image_point.distance / hypotenuse_small;
  point.x = -image_point.image_x * ratio;
  point.y = ratio;
  point.z = -image_point.image_z * ratio;

  point.timestamp = image_point.timestamp;
  point.intensity = image_point.intensity;
}
