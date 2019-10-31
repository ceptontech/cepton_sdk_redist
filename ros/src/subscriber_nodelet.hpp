#pragma once

#include <string>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <cepton_sdk_api.hpp>

#include "cepton_ros/SensorInformation.h"
#include "cepton_ros/common.hpp"
#include "cepton_ros/point.hpp"

namespace cepton_ros {

class SubscriberNodelet : public nodelet::Nodelet {
 public:
  void on_sensor_information(const SensorInformation::ConstPtr& msg);
  void on_points(const CeptonPointCloud::ConstPtr& point_cloud);

 protected:
  void onInit() override;

 private:
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle;

  ros::Subscriber sensor_information_subscriber;
  ros::Subscriber points_subscriber;
};
}  // namespace cepton_ros