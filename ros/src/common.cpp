#include "cepton_ros/common.hpp"

namespace cepton_ros {
namespace rosutil {

ros::Time from_usec(int64_t usec) {
  ros::Time stamp;
  stamp.sec = double(usec) * 1e-6;
  usec -= int64_t(double(stamp.sec) * 1e6);
  stamp.nsec = double(usec) * 1e3f;
  return stamp;
}

int64_t to_usec(const ros::Time &stamp) {
  int64_t usec = 0;
  usec += double(stamp.sec) * 1e6;
  usec += double(stamp.nsec) * 1e-3;
  return usec;
}

}  // namespace rosutil
}  // namespace cepton_ros