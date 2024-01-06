#pragma once

#include <grid_map_msgs/GridMap.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <voxel_grid_large.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <map_cleaner/DataLoader.hpp>
#include <map_cleaner/DivideByTerrain.hpp>
#include <map_cleaner/utils.hpp>
#include "ros/assert.h"

#include <pcl/octree/octree_search.h>
#include "kumo_structures.hpp"
class MovingPointIdentification {
 public:
  typedef std::shared_ptr<MovingPointIdentification> Ptr;
  typedef std::shared_ptr<const MovingPointIdentification> ConstPtr;

 private:
  const std::string layer_name_ = "range";

  float lidar_range_sq_;
  float fov_h_, fov_v_;
  float res_h_, res_v_;
  float res_h_scale_, res_v_scale_, range_im_res_;
  int delta_h_, delta_v_;
  bool spinning_lidar_;
  int frame_skip_;
  float threshold_;
  float submap_update_dist_;
  pcl::VoxelGridLarge<PointType> vg_;

  PublisherPtr pub_ptr_;
  tf2_ros::StaticTransformBroadcaster static_tf_br_;
  std::string frame_id_;
  pcl::octree::OctreePointCloudSearch<PointType>::Ptr octree_;

  std::unordered_map<int, kumo::VoxelKey> point2voxel_;
  std::unordered_map<int, kumo::GridKey> point2grid_;

  // kumo::VoxelMap voxel_map_;

  enum class ComparisonResult {
    CASE_A = 0,
    CASE_B = 1,
    CASE_C = 2,
    CASE_D = 3
  };
  enum class FusedResult { CASE_A, CASE_C, OTHERWISE };

  inline void getRangeImageCoordinate(const PointType &p, float &range,
                                      float &pos_h, float &pos_v) {
    range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    pos_h = std::atan2(p.y, p.x) * res_h_scale_;
    pos_v = std::asin(p.z / range) * res_v_scale_;
  }

  grid_map::GridMapPtr buildRangeImage(const CloudType &input,
                                       std::vector<bool> &is_above_ground_vec) {
    grid_map::GridMapPtr range_im(new grid_map::GridMap);
    range_im->setFrameId("map");
    range_im->setGeometry(
        grid_map::Length(fov_h_ * res_h_scale_, fov_v_ * res_v_scale_),
        range_im_res_, grid_map::Position(0.0, 0.0));
    range_im->add(layer_name_);

    grid_map::Matrix &layer = (*range_im)[layer_name_];
    for (int i = 0; i < input.size(); i++) {
      const PointType &p = input[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      if (!is_above_ground_vec[i]) {
        continue;
      }

      float range, pos_h, pos_v;
      getRangeImageCoordinate(p, range, pos_h, pos_v);

      grid_map::Index idx;
      if (!range_im->getIndex(grid_map::Position(pos_h, pos_v), idx)) {
        continue;
      }

      layer(idx[0], idx[1]) = range;
    }

    return range_im;
  }

  void buildSubMap(const CloudType &input,
                   const KDTreeNanoFlannPCL<PointType> &kdtree,
                   const Eigen::Affine3f &lidar_pose, CloudType &submap,
                   std::vector<int> &indices) {
    submap.clear();
    indices.clear();

    PointType center;
    center.x = lidar_pose.translation().x();
    center.y = lidar_pose.translation().y();
    center.z = lidar_pose.translation().z();

    std::vector<nanoflann::ResultItem<int, float>> nanoflann_res;
    kdtree.radiusSearch(center.data, lidar_range_sq_, nanoflann_res,
                        nanoflann::SearchParameters());

    submap.resize(nanoflann_res.size());
    indices.resize(nanoflann_res.size());
    for (int i = 0; i < submap.size(); ++i) {
      submap[i] = input[nanoflann_res[i].first];
      indices[i] = nanoflann_res[i].first;
    }

    // kumo: build octomap for submap
    // float octomap_resolution_ = 0.1;
    // octree_.reset(new pcl::octree::OctreePointCloudSearch<PointType>(
    //     octomap_resolution_));
    // octree_->setInputCloud(submap.makeShared());
    // octree_->addPointsFromInputCloud();
    // octree_->
  }

  inline ComparisonResult compareRange(const float scan_range,
                                       const float target_range) {
    if (!std::isfinite(scan_range) || !std::isfinite(target_range))
      return ComparisonResult::CASE_D;
    else if (target_range > scan_range + threshold_)
      return ComparisonResult::CASE_B;
    else if (target_range < scan_range - threshold_)
      return ComparisonResult::CASE_C;
    else
      return ComparisonResult::CASE_A;
  }

  inline FusedResult getFusedResult(const grid_map::GridMap &range_im,
                                    const grid_map::Matrix &layer,
                                    const grid_map::Position &target_pos,
                                    const float &target_range) {
    const grid_map::Size im_size = range_im.getSize();
    grid_map::Index target_idx;
    if (!range_im.getIndex(target_pos, target_idx))
      return FusedResult::OTHERWISE;

    int counter[4] = {0, 0, 0, 0};
    for (int d_v = -delta_v_; d_v <= delta_v_; ++d_v) {
      for (int d_h = -delta_h_; d_h <= delta_h_; ++d_h) {
        grid_map::Index idx(target_idx[0] + d_h, target_idx[1] + d_v);
        if (0 > idx[0]) {
          if (spinning_lidar_)
            idx[0] = im_size[0] + idx[0];
          else
            continue;
        }
        if (idx[0] >= im_size[0]) {
          if (spinning_lidar_)
            idx[0] = idx[0] - im_size[0];
          else
            continue;
        }
        if (0 > idx[1] || idx[1] >= im_size[1]) continue;

        float scan_range = layer(idx[0], idx[1]);
        ComparisonResult res = compareRange(scan_range, target_range);
        counter[(int)res]++;
      }
    }

    // FIXME : need to be modified
    if (counter[0] != 0)
      return FusedResult::CASE_A;
    else if (counter[1] != 0)
      return FusedResult::OTHERWISE;
    else if (counter[2] != 0)
      return FusedResult::CASE_C;
    // else if(counter[3] != 0)
    //   return FusedResult::CASE_C;

    return FusedResult::OTHERWISE;
  }

  void compareSubmapAndScan(const grid_map::GridMap &range_im,
                            const CloudType &submap,
                            const std::vector<int> &indices,
                            std::vector<int> &vote_list_static,
                            std::vector<int> &vote_list_dynamic) {
    const grid_map::Matrix &layer = range_im[layer_name_];

#pragma omp parallel for
    for (int i = 0; i < submap.size(); ++i) {
      float range, pos_h, pos_v;
      getRangeImageCoordinate(submap[i], range, pos_h, pos_v);
      FusedResult res = getFusedResult(range_im, layer,
                                       grid_map::Position(pos_h, pos_v), range);
      if (res == FusedResult::CASE_A)
        vote_list_static[indices[i]]++;
      else if (res == FusedResult::CASE_C)
        vote_list_dynamic[indices[i]]++;
    }
  }

  void KumoCompareSubmapWithScan(const grid_map::GridMap &range_im,
                                 const CloudType &submap,
                                 const std::vector<int> &indices,
                                 std::vector<int> &vote_list_static,
                                 std::vector<int> &vote_list_dynamic, kumo::VoxelMap& voxel_map) {
    const grid_map::Matrix &layer = range_im[layer_name_];

#pragma omp parallel for
    for (int i = 0; i < submap.size(); ++i) {
      float range, pos_h, pos_v;
      getRangeImageCoordinate(submap[i], range, pos_h, pos_v);
      FusedResult res = getFusedResult(range_im, layer,
                                       grid_map::Position(pos_h, pos_v), range);

      if (res == FusedResult::CASE_A)
        vote_list_static[indices[i]]++;
      else if (res == FusedResult::CASE_C)
        vote_list_dynamic[indices[i]]++;
      // DeepDarkFantasies(submap, submap[i], indices[i]);
    }
  }

  inline void DeepDarkFantasies(const CloudType &submap,
                                const PointType &pt_submap,
                                int original_index) {
    // kumo::VoxelKey &key = point2voxel_[original_index];
    // auto& voxel = voxel_map_[key];
    // get voxel point indices
    // for (int i = 0; i < voxel.indices.size(); ++i) {
    // int index = voxel.indices[i];
    // if (index == original_index) continue;
    // const PointType &pt = submap[index];
    // }
    // 计算立方体的边界
    // float half_side = 0.6;  // 立方体半边长
    // Eigen::Vector3f minPoint(pt_submap.x - half_side,
    //                          pt_submap.y - half_side,
    //                          pt_submap.z - half_side);
    // Eigen::Vector3f maxPoint(pt_submap.x + half_side,
    //                          pt_submap.y + half_side,
    //                          pt_submap.z + half_side);
    // // 执行 boxSearch
    // std::vector<int> pointIdxBoxSearch;
    // int num_searched = octree_->boxSearch(minPoint, maxPoint,
    // pointIdxBoxSearch);
    // // ROS_WARN("num_searched: %d", num_searched);
  }


  void NormalAwareCompareSubmapWithScan() {

  }


  void publish(const DataLoaderBase::Frame &frame,
               const CloudType::ConstPtr &input,
               const std::vector<int> &indices,
               const std::vector<int> &vote_list_static,
               const std::vector<int> &vote_list_dynamic) {
    if (pub_ptr_ == nullptr) return;

    pcl::PointCloud<pcl::PointXYZRGB> vis_cloud;
    vis_cloud.reserve(indices.size() + frame.frame->size());

    CloudType transformed_cloud;
    pcl::transformPointCloud(*frame.frame, transformed_cloud,
                             Eigen::Translation3f(frame.t) * frame.r.matrix());
    for (int i = 0; i < transformed_cloud.size(); i++) {
      pcl::PointXYZRGB vis_p;
      const PointType &p = transformed_cloud[i];
      vis_p.x = p.x;
      vis_p.y = p.y;
      vis_p.z = p.z;
      vis_p.r = 200;
      vis_p.g = 200;
      vis_p.b = 200;

      vis_cloud.push_back(vis_p);
    }

    for (int i = 0; i < indices.size(); i++) {
      int idx = indices[i];
      pcl::PointXYZRGB vis_p;
      const PointType &p = input->points[idx];
      vis_p.x = p.x;
      vis_p.y = p.y;
      vis_p.z = p.z;

      if (vote_list_dynamic[idx] <= vote_list_static[idx]) {
        vis_p.r = 0;
        vis_p.g = 255;
        vis_p.b = 255;
      } else {
        vis_p.r = 255;
        vis_p.g = 0;
        vis_p.b = 0;
      }

      vis_cloud.push_back(vis_p);
    }

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(vis_cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = ros::Time::now();
    pub_ptr_->publish(cloud_msg);

    geometry_msgs::TransformStamped static_tf;
    static_tf.header.stamp = ros::Time::now();
    static_tf.header.frame_id = frame_id_;
    static_tf.child_frame_id = "lidar";
    static_tf.transform.translation.x = frame.t[0];
    static_tf.transform.translation.y = frame.t[1];
    static_tf.transform.translation.z = frame.t[2];
    static_tf.transform.rotation.x = frame.r.x();
    static_tf.transform.rotation.y = frame.r.y();
    static_tf.transform.rotation.z = frame.r.z();
    static_tf.transform.rotation.w = frame.r.w();
    static_tf_br_.sendTransform(static_tf);
  }

 public:
  MovingPointIdentification(const float voxel_leaf_size, const float res_h,
                            const float res_v, const float fov_h,
                            const float fov_v, const float lidar_range,
                            const int delta_h, const int delta_v,
                            const float threshold, const int frame_skip,
                            const float submap_update_dist,
                            const PublisherPtr pub_ptr = nullptr,
                            const std::string &frame_id = "map") {
    vg_.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    octree_.reset(
        new pcl::octree::OctreePointCloudSearch<PointType>(voxel_leaf_size));
    fov_h_ = fov_h;
    fov_v_ = fov_v;

    if (fov_h_ > M_PI * 2.0) fov_h_ = M_PI * 2.0;

    if (fov_v_ > M_PI) fov_v_ = M_PI;

    res_h_ = res_h;
    res_v_ = res_v;

    if ((M_PI * 2.0 - fov_h_) < res_h_)
      spinning_lidar_ = true;
    else
      spinning_lidar_ = false;

    if (res_h_ < res_v_) {
      res_h_scale_ = 1.0;
      res_v_scale_ = res_h_ / res_v_;
      range_im_res_ = res_h_;
    } else {
      res_h_scale_ = res_v_ / res_h_;
      res_v_scale_ = 1.0;
      range_im_res_ = res_v_;
    }
    lidar_range_sq_ = lidar_range * lidar_range;
    delta_h_ = delta_h;
    delta_v_ = delta_v;
    threshold_ = threshold;
    frame_skip_ = frame_skip;
    submap_update_dist_ = submap_update_dist;

    pub_ptr_ = pub_ptr;
    frame_id_ = frame_id;
  }

  void InsertNonground2GridAndVoxelMaps(kumo::VoxelMap &voxel_map,
                                        kumo::GridMap &grid_map,
                                        CloudType::Ptr cloud) {
    static float voxel_size_ = 0.8;
    for (int i = 0; i < cloud->size(); i++) {
      const PointType &p = cloud->points[i];
      voxel_map.insertPointIntoVoxelMap(p, i);
      grid_map.insertPointIntoGridMap(p, i, false);
    }
  }

  void InsertGround2GridMap(kumo::GridMap &grid_map, CloudType::Ptr cloud) {
    for (int i = 0; i < cloud->size(); i++) {
      const PointType &p = cloud->points[i];
      grid_map.insertPointIntoGridMap(p, i, true);
    }
  }

  bool compute(DataLoaderBase::Ptr &loader,
               DivideByTerrain::Ptr divide_by_terrain,
               const grid_map::GridMap &grid_terrain,
               const CloudType::ConstPtr &cloud,
               const PIndices::ConstPtr &aboveground_indices,
               const PIndices::ConstPtr &ground_indices,
               PIndices &static_indices, PIndices &dynamic_indices, /*||*/
               PIndices &extra_dynamic_indices_raw) {
    if (cloud->empty() || loader->getSize() == 0 ||
        aboveground_indices->indices.empty())
      return false;

    CloudType::Ptr cloud_target(new CloudType);
    std::vector<std::vector<int>> vg_indices;
    vg_.setInputCloud(cloud);
    vg_.setIndices(aboveground_indices);
    vg_.filterWithOutputIndices(*cloud_target, vg_indices);

    CloudType::Ptr cloud_ground_ds(new CloudType);
    std::vector<std::vector<int>> groundds_indices;
    vg_.setInputCloud(cloud);
    vg_.setIndices(ground_indices);
    vg_.filterWithOutputIndices(*cloud_ground_ds, groundds_indices);

    // kumo: build voxelmap and gridmap
    kumo::GridMap kumo_grid_map(cloud_target);
    kumo_grid_map.InitGridTerrainHeights(grid_terrain, "elevation");
    InsertGround2GridMap(kumo_grid_map, cloud_ground_ds);

    kumo::VoxelMap voxel_map_(cloud_target);
    InsertNonground2GridAndVoxelMaps(voxel_map_, kumo_grid_map, cloud_target);
    // kumo_grid_map.DEBUGGetGroundNum();
    // ROS_BREAK();

    ROS_ERROR("original cloud size: %d", cloud->size());
    ROS_ERROR("ds cloud size: %d", cloud_target->size());
    ROS_ERROR("kumo: voxel map size: %d, total record points: %d", voxel_map_.num_valid_voxels_,
              voxel_map_.num_points_total_);
    ROS_ERROR("kumo: grid map size: %d, total record points: %d",
              kumo_grid_map.num_valid_grids_, kumo_grid_map.num_points_total_);

    // original
    std::vector<int> vote_list_static;
    vote_list_static.resize(cloud_target->size(), 0);
    std::vector<int> vote_list_dynamic;
    vote_list_dynamic.resize(cloud_target->size(), 0);

    PointCloudAdaptor<PointType> adapter;
    adapter.cloud_ptr_ = cloud_target;
    KDTreeNanoFlannPCL<PointType> kdtree(3, adapter);
    kdtree.buildIndex();

    CloudType::Ptr submap(new CloudType);
    CloudType::Ptr lidar_coordinate_submap(new CloudType);
    std::vector<int> submap_indices;
    submap_indices.reserve(cloud_target->size());
    Eigen::Affine3f last_lidar_pose = Eigen::Affine3f::Identity();

    for (int i = 0; i < loader->getSize(); ++i) {
      auto st = ros::WallTime::now();
      DataLoaderBase::Frame frame = loader->loadFrame(i);
      if (frame.frame->empty()) {
        ROS_WARN_STREAM("Frame " << frame.idx + 1 << " is empty.");
        continue;
      }
      // std::cout << "load: " << (ros::WallTime::now() - st).toSec() << " sec"
      // << std::endl;

      if (i % (frame_skip_ + 1) != 0) continue;

      //////////////////////////////////////////////////////////////////////////////////////////
      // kumo: filter belowground part
      /// kumo: transform frame to world frame to compare with terrain
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      T.block<3, 3>(0, 0) = frame.r.toRotationMatrix();
      T.block<3, 1>(0, 3) = frame.t;
      CloudType::Ptr cloud_frame_world(new CloudType);
      pcl::transformPointCloud(*frame.frame, *cloud_frame_world, T);

      std::vector<bool> is_above_ground_vec(frame.frame->size(), false);
      if (!divide_by_terrain->compute(*cloud_frame_world, grid_terrain,
                                      is_above_ground_vec)) {
        ROS_ERROR("Failed to divide by terrain for scan.");
        ROS_BREAK();
      }
      // st = ros::WallTime::now(); //TODO (kumo) we can put the aboveground
      // filter inside the buildRangeImage function
      grid_map::GridMapPtr range_im =
          buildRangeImage(*frame.frame, is_above_ground_vec);
      // std::cout << "build range image: " << (ros::WallTime::now() -
      // st).toSec() << " sec" << std::endl;

      ////////////////////////////////////build
      ///submap////////////////////////////////////////////
      Eigen::Affine3f lidar_pose =
          Eigen::Translation3f(frame.t) * frame.r.matrix();
      if (submap->empty() || (last_lidar_pose.translation() - frame.t).norm() >
                                 submap_update_dist_) {
        // st = ros::WallTime::now();
        buildSubMap(*adapter.cloud_ptr_, kdtree, lidar_pose, *submap,
                    submap_indices);
        last_lidar_pose = lidar_pose;
        // std::cout << "build sub map: " << (ros::WallTime::now() - st).toSec()
        // << " sec" << std::endl;
      }
      pcl::transformPointCloud(*submap, *lidar_coordinate_submap,
                               lidar_pose.inverse());

      st = ros::WallTime::now();
      ////////////////////////////////////compare////////////////////////////////////////////
      // compareSubmapAndScan(*range_im, *lidar_coordinate_submap,
      // submap_indices, vote_list_static, vote_list_dynamic);
      KumoCompareSubmapWithScan(*range_im, *lidar_coordinate_submap,
                                submap_indices, vote_list_static,
                                vote_list_dynamic, voxel_map_);
      std::cout << "compare: " << (ros::WallTime::now() - st).toSec() << " sec"
                << std::endl;

      publish(frame, cloud_target, submap_indices, vote_list_static,
              vote_list_dynamic);

      if (i % 100 == 0) {
        ROS_INFO_STREAM("Compute Moving Point Identification: "
                        << i + 1 << " / " << loader->getSize());
      }
    }

    DataLoaderBase::Frame dummy_frame;
    dummy_frame.t = loader->getFrameInfo(loader->getSize() - 1).t;
    dummy_frame.r = loader->getFrameInfo(loader->getSize() - 1).r;
    publish(dummy_frame, cloud_target, std::vector<int>(), vote_list_static,
            vote_list_dynamic);  // dummy
    ROS_INFO_STREAM("Compute Moving Point Identification: "
                    << loader->getSize() << " / " << loader->getSize());

    static_indices.indices.clear();
    dynamic_indices.indices.clear();
    std::vector<bool> is_dynamic_vec(cloud_target->size(), false);
    std::vector<int> dynamic_indices_ds, static_indices_ds;
    for (int i = 0; i < cloud_target->size(); i++) {
      std::vector<int> *dst_indices;
      if (vote_list_dynamic[i] > vote_list_static[i]) {  // dynamic
        is_dynamic_vec[i] = true;
        dynamic_indices_ds.push_back(i);
      }
      // else {// dynamic
      //   dst_indices = &dynamic_indices_ds;
      //   is_dynamic_vec[i] = true;
      // }
      // for(int j = 0; j < vg_indices[i].size(); ++j){
      //   dst_indices->indices.push_back(vg_indices[i][j]);
      // }
    }

    // expand dynamic indices
    ROS_WARN("dynamic_indices_ds size: %d", dynamic_indices_ds.size());
    for (const auto &index : dynamic_indices_ds) {
      kumo_grid_map.IncreaseDynamicNums(index);
    }
    // kumo_grid_map.DEBUGGetGroundNum();
    // ROS_BREAK();

    // std::vector<int> extra_dynamic_indices =
    std::vector<int> extra_dynamic_indices = kumo_grid_map.NeighborAwareRegionGrowing();
    ROS_WARN("extra_dynamic_indices size: %d", extra_dynamic_indices.size());
    // std::vector<int> extra_dynamic_indices =
    // kumo_grid_map.GetExpandGridDynamicIndices();

    // 1. final dynamic = dynamic + extra_dynamic --> retreive voxels
    dynamic_indices_ds.insert(dynamic_indices_ds.end(),
                              extra_dynamic_indices.begin(),
                              extra_dynamic_indices.end());
    /// unordered set 去重dynamic indices
    std::unordered_set<int> dynamic_indices_set(dynamic_indices_ds.begin(),
                                                dynamic_indices_ds.end());
    dynamic_indices_ds.assign(dynamic_indices_set.begin(),
                              dynamic_indices_set.end());

    /// get raw dynamic
    for (const auto &idx_ds : dynamic_indices_ds) {
      for (int idx_raw = 0; idx_raw < vg_indices[idx_ds].size(); ++idx_raw) {
        dynamic_indices.indices.push_back(vg_indices[idx_ds][idx_raw]);
      }
    }

    // 2. final static = static - extra_dynamic --> retreive voxels
    for (int i = 0; i < cloud_target->size(); i++) {
      if (is_dynamic_vec[i]) {
        continue;
      }
      /// get raw static
      for (int idx_raw = 0; idx_raw < vg_indices[i].size(); ++idx_raw) {
        static_indices.indices.push_back(vg_indices[i][idx_raw]);
      }
    }

    static_indices.indices.shrink_to_fit();
    dynamic_indices.indices.shrink_to_fit();


    // for viz
    for (const auto &idx_ds : extra_dynamic_indices) {
      for (int idx_raw = 0; idx_raw < vg_indices[idx_ds].size(); ++idx_raw) {
        extra_dynamic_indices_raw.indices.push_back(
            vg_indices[idx_ds][idx_raw]);
      }
    }
    return true;
  }
};