#pragma once

#include <cmath>

struct CompiledSensorTransform {
  float translation[3];

  float rotation_m00;
  float rotation_m01;
  float rotation_m02;
  float rotation_m10;
  float rotation_m11;
  float rotation_m12;
  float rotation_m20;
  float rotation_m21;
  float rotation_m22;

  void apply(float &x, float &y, float &z) {
    float x_tmp = x * rotation_m00 + y * rotation_m01 + z * rotation_m02;
    float y_tmp = x * rotation_m10 + y * rotation_m11 + z * rotation_m12;
    float z_tmp = x * rotation_m20 + y * rotation_m21 + z * rotation_m22;

    x_tmp += translation[0];
    y_tmp += translation[1];
    z_tmp += translation[2];

    x = x_tmp;
    y = y_tmp;
    z = z_tmp;
  }
};

CompiledSensorTransform compile_sensor_transform(
    const std::array<float, 3> &translation,
    const std::array<float, 4> &rotation) {
  CompiledSensorTransform compiled_transform;
  compiled_transform.translation = translation;

  float x = rotation[0];
  float y = rotation[1];
  float z = rotation[2];
  float w = rotation[3];
  float xx = x * x;
  float xy = x * y;
  float xz = x * z;
  float xw = x * w;
  float yy = y * y;
  float yz = y * z;
  float yw = y * w;
  float zz = z * z;
  float zw = z * w;

  compiled_transform.rotation_m00 = 1 - 2 * (yy + zz);
  compiled_transform.rotation_m01 = 2 * (xy - zw);
  compiled_transform.rotation_m02 = 2 * (xz + yw);

  compiled_transform.rotation_m10 = 2 * (xy + zw);
  compiled_transform.rotation_m11 = 1 - 2 * (xx + zz);
  compiled_transform.rotation_m12 = 2 * (yz - xw);

  compiled_transform.rotation_m20 = 2 * (xz - yw);
  compiled_transform.rotation_m21 = 2 * (yz + xw);
  compiled_transform.rotation_m22 = 1 - 2 * (xx + yy);

  return compiled_transform;
}
