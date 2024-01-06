#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxel_grid_large.h>
#include "utils.hpp"

namespace kumo {
void KPubCloud(const CloudType::ConstPtr &cloud,
                  const PIndices::ConstPtr &indices, const std::string frame_id, const float vis_vg_res,
                  ros::Publisher &pub) {
  pcl::VoxelGridLarge<PointType> vis_vg_;
  vis_vg_.setLeafSize(vis_vg_res, vis_vg_res, vis_vg_res);

  CloudType::Ptr vis_cloud(new CloudType);
  if (indices != nullptr) {
    vis_vg_.setInputCloud(cloud);
    vis_vg_.setIndices(indices);
    vis_vg_.filter(*vis_cloud);
  } else {
    *vis_cloud = *cloud;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*vis_cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  pub.publish(cloud_msg);
}
}  // namespace kumo