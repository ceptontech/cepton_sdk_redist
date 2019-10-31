#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#define FATAL_ERROR(error)         \
  do {                             \
    if (error) {                   \
      NODELET_FATAL(error.what()); \
      return;                      \
    }                              \
  } while (0)

#define WARN_ERROR(error)         \
  do {                            \
    if (error) {                  \
      NODELET_WARN(error.what()); \
      return;                     \
    }                             \
  } while (0)

namespace cepton_ros {
namespace rosutil {

ros::Time from_usec(int64_t usec);
int64_t to_usec(const ros::Time &stamp);

}  // namespace rosutil
}  // namespace cepton_ros