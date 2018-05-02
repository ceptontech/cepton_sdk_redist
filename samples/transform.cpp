#include "cepton_sdk.hpp"
#include "cepton_sdk_util.hpp"

int main() {
  // Create transform
  std::array<float, 3> translation = {0.0f, 0.0f, 1.0f};
  std::array<float, 4> rotation = {1.0f, 0.0f, 0.0f, 0.0f};
  auto compiled_transform = cepton_sdk::CompiledTransform::create(
      translation.data(), rotation.data());

  // Apply transform
  cepton_sdk::SensorPoint point = {};
  compiled_transform.apply(point.x, point.y, point.z);
  printf("Point: [%f, %f, %f]\n", point.x, point.y, point.z);
}
