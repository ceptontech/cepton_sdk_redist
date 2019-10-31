#include "subscriber_nodelet.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cepton_ros::SubscriberNodelet, nodelet::Nodelet);

namespace cepton_ros {

void SubscriberNodelet::on_sensor_information(
    const SensorInformation::ConstPtr& msg) {}

void SubscriberNodelet::on_points(
    const CeptonPointCloud::ConstPtr& point_cloud) {
  const auto& points = point_cloud->points;
  if (points.size() == 0) return;
  NODELET_INFO("Received %i points.", (int)points.size());
  NODELET_INFO("  Timestamp Range: [%li, %li]", (long int)points[0].timestamp,
               (long int)points[points.size() - 1].timestamp);

  // Print info for first valid point
  int i;
  for (i = 0; i < points.size(); ++i) {
    if (points[i].valid) break;
  }
  const auto& point = points[i];
  NODELET_INFO("  Point %i:", i);
  NODELET_INFO("    Image Position: [%.3f, %.3f]", point.image_x, point.image_z);
  NODELET_INFO("    Distance: %.2f", point.distance);
  NODELET_INFO("    Position: [%.2f, %.2f, %.2f]", point.x, point.y, point.z);
  NODELET_INFO("    Intensity: %.2f", point.intensity);
  NODELET_INFO("    Return Type: %i", (int)point.return_type);
  NODELET_INFO("    Flags: %i", (int)point.flags);
}

void SubscriberNodelet::onInit() {
  this->node_handle = getNodeHandle();
  this->private_node_handle = getPrivateNodeHandle();

  sensor_information_subscriber = node_handle.subscribe<SensorInformation>(
      "cepton/sensor_information", 10,
      &SubscriberNodelet::on_sensor_information, this);
  points_subscriber = node_handle.subscribe<CeptonPointCloud>(
      "cepton/points", 10, &SubscriberNodelet::on_points, this);
}
}  // namespace cepton_ros