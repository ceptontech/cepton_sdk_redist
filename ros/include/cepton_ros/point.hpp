#pragma once

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cepton_sdk_util.hpp>

namespace cepton_ros {
using CeptonPointCloud = pcl::PointCloud<cepton_sdk::util::SensorPoint>;
}  // namespace cepton_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(cepton_sdk::util::SensorPoint,
    (double, timestamp, timestamp) // int64_t not supported in PCL
    (float, image_x, image_x)
    (float, distance, distance)
    (float, image_z, image_z)
    (float, intensity, intensity)
    (uint8_t, return_type, return_type)
    (uint8_t, flags, flags)
    (float, x, x)
    (float, y, y)
    (float, z, z)
  )
// clang-format on
