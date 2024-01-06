#pragma once
// strange data structures
// #include <opencv2/imgproc.hpp>
#include <cassert>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen/src/Core/Matrix.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include <Eigen/Dense>
#include <grid_map_ros/grid_map_ros.hpp>
#include <map_cleaner/utils.hpp>
#include <queue>
#include <unordered_map>
#include <vector>

namespace kumo {
/////////////////////////////////cell/////////////////////////////////////
typedef Eigen::Vector2i GridKey;
enum class GridStatus {
  KUNKNOWN = 0,
  EXPANDABLE = 1,
  NOT_EXPANDABLE = 2,
  EXPANDED = 3
};
struct Grid {

  float x_center = std::numeric_limits<float>::quiet_NaN(),
        y_center = std::numeric_limits<float>::quiet_NaN();
  GridKey self_key;
  std::vector<int> ground_indices;
  std::vector<int> nonground_indices;
  std::vector<int> dynamic_indices;
  int num_points_total = 0;
  int num_ground = 0;
  int num_dynamic = 0;
  float z_terrain = 0.0;
  bool is_valid = false;
  enum GridStatus status = GridStatus::KUNKNOWN;
};
// 定义一个2D向量作为哈希表的键
// typedef Eigen::Vector2i GridKey;

// 自定义哈希函数
struct GridHash {
  std::size_t operator()(const GridKey& k) const {
    return std::hash<int>()(k[0]) ^ std::hash<int>()(k[1]) << 1;
  }
};

typedef std::unordered_map<GridKey, Grid, GridHash> GridMapStructure;

struct GridMap {
  GridMapStructure grid_map_;
  std::unordered_map<int, kumo::GridKey> point2grid_;
  float grid_size_ = 0.8;
  int thr_min_ground_num_ = 5;
  float max_judge_height_ = 3.0;
  float thr_expand_dynamic_ratio_ = 0.8;
  float thr_expand_ground_ratio_ = 0.7;

  int num_valid_grids_ = 0;
  int num_points_total_ = 0;
  bool is_terrain_init_ = false;
  float thr_distance_growing_ = 0.2;
  pcl::PointCloud<PointType>::ConstPtr cloud_;

  // GridMap(const float& grid_size) : grid_size_(grid_size) {}
  GridMap(pcl::PointCloud<PointType>::ConstPtr cloud) : cloud_(cloud) {}

  void InitGridTerrainHeights(const grid_map::GridMap& grid_terrain,
                              std::string input_layer_name) {
    if (!grid_terrain.exists(input_layer_name)) {
      ROS_ERROR_STREAM("GridMap Does Not Have The Required Layers.");
      ROS_BREAK();
    }
    const grid_map::Matrix& terrain_layer = grid_terrain[input_layer_name];

    int DEBUG_num_invalid_grid = 0;
    for (auto& grid : grid_map_) {
      auto& grid_data = grid.second;
      if (!grid_data.is_valid) {
        DEBUG_num_invalid_grid += 1;
        continue;
      }
      // get terrain map index
      grid_map::Index idx;
      if (!grid_terrain.getIndex(
              grid_map::Position(grid_data.x_center, grid_data.y_center),
              idx)) {
        ROS_ERROR("grid_terrain.getIndex failed, position: (%f, %f)",
                  grid_data.x_center, grid_data.y_center);
        continue;
      }
      // set kumo grid terrain height
      grid_data.z_terrain = terrain_layer(idx[0], idx[1]);
    }
    is_terrain_init_ = true;
    ROS_WARN("DEBUG_num_invalid_grid: %d", DEBUG_num_invalid_grid);
  }

  void insertPointIntoGridMap(const PointType point, const int& index,
                              bool is_ground) {
    // 计算网格键
    GridKey key;
    key[0] = static_cast<int>(floor(point.x / grid_size_));
    key[1] = static_cast<int>(floor(point.y / grid_size_));

    // 插入点索引到哈希表，并更新网格数据
    auto& grid = grid_map_[key];
    ROS_WARN_ONCE("DEBUG: z_terrain %f, point.z %f", grid.z_terrain, point.z);
    if (point.z > grid.z_terrain + max_judge_height_) {
      return;
    }
    if (!grid.is_valid) {
      grid.is_valid = true;
      grid.x_center = key[0] * grid_size_ + grid_size_ / 2.0;
      grid.y_center = key[1] * grid_size_ + grid_size_ / 2.0;
      num_valid_grids_ += 1;
    }
    // 更新其他网格数据，例如 num_points_total，num_ground 等
    // if (point.z < grid.z_terrain + max_judge_height_) {
    // }
    if (is_ground) {
      grid.num_ground += 1;
    } else {
      grid.nonground_indices.push_back(index);
    }
    grid.num_points_total += 1;
    point2grid_[index] = key;
    num_points_total_ += 1;
    // ...
  }

  void DEBUGGetGroundNum() {
    // return;
    for (auto& grid : grid_map_) {
      auto& grid_data = grid.second;
      if (!grid_data.is_valid) {
        continue;
      }
      const int num_ground = grid_data.num_ground;

      ROS_WARN(
          "grid pos: %f,%f, num_ground: %d, num_points_total: %d, num_nonground: %d, num_dynamic: %d",
          grid_data.x_center, grid_data.y_center, num_ground,
          grid_data.num_points_total,
          grid_data.nonground_indices.size(), grid_data.num_dynamic);
      assert(grid_data.num_points_total >= grid_data.num_dynamic);
      assert(grid_data.num_points_total ==
             grid_data.nonground_indices.size() + num_ground);

      ROS_WARN("====================================== >>>>>>>>>");
      ROS_WARN("debug grid: nonground %d, dy %d",
               grid_data.nonground_indices.size(),
               grid_data.dynamic_indices.size());
      ROS_WARN("====================================== >>>>>>>>>");

      // 因为地图存的nonground是filter了height的，而更新dynamic没限制height，可能出现记录Nonground < dynamic，（z thresh之上出现较多动态点) 应该不用担心
    }
  }

  inline void IncreaseDynamicNums(int index) {
    auto& grid = grid_map_[point2grid_[index]];
    grid.num_dynamic += 1;
    grid.dynamic_indices.push_back(index);
  }

  // old
  std::vector<int> GetExpandGridDynamicIndices() {
    if (!is_terrain_init_) {
      ROS_ERROR("Terrain Height Has Not Been Initialized.");
      ROS_BREAK();
    }
    int DEBUG_num_expand_grid = 0;
    std::vector<int> dynamic_points;
    for (auto& grid : grid_map_) {
      const auto& grid_data = grid.second;
      if (!grid_data.is_valid) {
        continue;
      }
      const int num_dynamic = grid_data.num_dynamic;
      const int num_ground = grid_data.num_ground;
      const int num_nonground = grid_data.num_points_total - num_ground;
      const int num_nondynamic = grid_data.num_points_total - num_dynamic;
      if (num_nonground == 0) {
        continue;
      }
      if (num_dynamic > 0) {
        ROS_WARN(
            "grid pos: %f,%f, num_dynamic: %d, num_ground: %d, num_nonground: "
            "%d, num_nondynamic: %d",
            grid_data.x_center, grid_data.y_center, num_dynamic, num_ground,
            num_nonground, num_nondynamic);
      }

      if (num_nondynamic == 0 ||
          1.0 * num_dynamic / num_nonground > thr_expand_dynamic_ratio_ &&
              1.0 * num_ground / num_nondynamic > thr_expand_ground_ratio_) {
        dynamic_points.insert(dynamic_points.end(),
                              grid_data.nonground_indices.begin(),
                              grid_data.nonground_indices.end());
        DEBUG_num_expand_grid++;
      }
    }
    ROS_WARN("expand dynamic grid num: %d", DEBUG_num_expand_grid);
    return dynamic_points;
  }

  //
  std::vector<Grid*> getNeighboringGrids(const GridKey& key) {
    std::vector<Grid*> neighbors;
    // 定义邻居的相对位置（上，下，左，右）
    std::vector<GridKey> neighbor_offsets = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (const auto& offset : neighbor_offsets) {
      GridKey neighbor_key = key + offset;  // 计算邻居的键值
      auto it = grid_map_.find(neighbor_key);
      if (it != grid_map_.end()) {
        if (it->second.is_valid) {
          neighbors.push_back(&(it->second));  // 如果邻居存在，添加到列表
        }
      }
    }
    return neighbors;
  }

  bool getExpandableNeighboringGrids(const GridKey& key,
                                     std::vector<Grid*>& neighbor_grids) {
    std::vector<Grid*> neighbors;
    // 定义邻居的相对位置（上，下，左，右）
    std::vector<GridKey> neighbor_offsets = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    bool has_unexpandable_neighbor = false;
    for (const auto& offset : neighbor_offsets) {
      GridKey neighbor_key = key + offset;  // 计算邻居的键值
      auto it = grid_map_.find(neighbor_key);
      if (it != grid_map_.end()) {
        if (IsGridExpandable(it->second) && it->second.is_valid) {
          neighbors.push_back(&(it->second));  // 如果邻居存在，添加到列表
        } else if (!IsGridExpandable(it->second)){
          has_unexpandable_neighbor = true;
          break;  // TODO 如果是边界或是遮挡到边界，可能会导致无法扩展
        }
      }
    }
    return !has_unexpandable_neighbor;
  }

  bool IsGridExpandable(Grid& grid) {
    // use cache
    if (grid.status == GridStatus::EXPANDABLE) {
      return true;
    } else if (grid.status == GridStatus::NOT_EXPANDABLE) {
      return false;
    }

    const int num_dynamic = grid.num_dynamic;
    const int num_ground = grid.num_ground;
    const int num_nonground = grid.num_points_total - num_ground;
    const int num_nondynamic = grid.num_points_total - num_dynamic;

    if (num_nondynamic == 0 ||
        1.0 * num_ground / num_nondynamic >
            thr_expand_ground_ratio_) {  // TODO 这个不应该用比值，应该用数量？
      grid.status = GridStatus::EXPANDABLE;
      return true;
    } else {
      grid.status = GridStatus::NOT_EXPANDABLE;
      return false;
    }
  }

  bool IsGridDynamic(Grid& grid) {
    if (!IsGridExpandable(grid)) {
      return false;
    }

    const int num_dynamic = grid.num_dynamic;
    const int num_ground = grid.num_ground;
    const int num_nonground = grid.num_points_total - num_ground;
    const int num_nondynamic = grid.num_points_total - num_dynamic;

    if (num_nonground == 0) {
      return false;
    }

    if (1.0 * num_dynamic / num_nonground >
        thr_expand_dynamic_ratio_) {  // TODO 这个不应该用比值，应该用数量？
      return true;
    } else {
      return false;
    }
  }

  void PerformRegionGrowing(Grid& current_grid,
                            std::vector<Grid*>& neighbor_grids,
                            pcl::PointCloud<PointType>::ConstPtr cloud_in,
                            std::vector<int>& new_dynamic_indices) {
    // prepare data
    std::vector<int> nonground_indices;   // 外部索引
    std::vector<int> dynamic_indices;  // 外部索引

    std::unordered_map<int, bool> dynamic_mapping;

    pcl::PointCloud<PointType> cloud_to_grow;  // 临时点云
    /// collect indices and points

    //// add current indices
    nonground_indices.insert(nonground_indices.end(),
                          current_grid.nonground_indices.begin(),
                          current_grid.nonground_indices.end());
    for (auto& idx : current_grid.nonground_indices) {
      cloud_to_grow.push_back(cloud_in->points[idx]);
    }
// ROS_WARN("\n====== >>>>>>>>>");
// ROS_WARN("current grid: nonground %d, dy %d", current_grid.nonground_indices.size(), current_grid.dynamic_indices.size());

    dynamic_indices.insert(dynamic_indices.end(),
                           current_grid.dynamic_indices.begin(),
                           current_grid.dynamic_indices.end());

    /// add neighbors' indices
    for (auto& neighbor_grid : neighbor_grids) {
      // ROS_WARN("neighbor_grid: nonground %d, dy %d",
      //          neighbor_grid->nonground_indices.size(),
      //          neighbor_grid->dynamic_indices.size());

      nonground_indices.insert(nonground_indices.end(),
                            neighbor_grid->nonground_indices.begin(),
                            neighbor_grid->nonground_indices.end());
      for (auto& idx : neighbor_grid->nonground_indices) {
        cloud_to_grow.push_back(cloud_in->points[idx]);
      }
      dynamic_indices.insert(dynamic_indices.end(),
                             neighbor_grid->dynamic_indices.begin(),
                             neighbor_grid->dynamic_indices.end());
    }
    // ROS_WARN("====== >>>>>>>>>\n");

    assert(nonground_indices.size() == cloud_to_grow.size());
    /// record dynamic，使用外部索引查询是否动态点
    for (auto& idx : dynamic_indices) {
      dynamic_mapping[idx] = true;
    }

    //////////////////////////////////////////
    // 开始生长
    std::queue<int> queue;  // 存储待处理点的索引，外部索引
    std::unordered_set<int> processed;  // 标记已处理的点， 临时索引

    // build kd tree
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud_to_grow.makeShared());

    // 初始化队列
    for (int index : dynamic_indices) {
      // FBI waring:
      // 这里的index是外部索引，相对于cloud_in，而不是cloud_to_grow
      queue.push(index);
      processed.insert(index);
    }

    // start
    while (!queue.empty()) {
      int current_index = queue.front();
      queue.pop();
      const auto& current_point = cloud_in->points[current_index];

      // 获取邻近点
      // raidus search
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if (kdtree.radiusSearch(current_point, thr_distance_growing_,
                              pointIdxRadiusSearch,
                              pointRadiusSquaredDistance) == 0) {
        continue;
      }

      // neighbor_index为临时索引，相对于cloud 或static indices
      // (因为cloud_to_grow 按static顺序构建)
      for (const auto& neighbor_index : pointIdxRadiusSearch) {
        if (processed.find(neighbor_index) != processed.end())
          continue;  // 如果已处理，跳过

        // 用neighbor临时找外部索引
        int& idx_original = nonground_indices[neighbor_index];
        if (dynamic_mapping.find(idx_original) != dynamic_mapping.end()) {
          // 如果邻居是原始动态点，pass
          continue;
        }

        // get new dynamic
        new_dynamic_indices.push_back(
            idx_original);         // 添加对应的外部索引到动态vec
        queue.push(idx_original);  // 添加对应的外部索引到队列
        processed.insert(neighbor_index);  // 将临时索引标记为已处理
      }
    }

    // ROS_WARN("growing cloud size: %d, original nondynamic: %d, expand num: %d",
    //          cloud_to_grow.size(), nonground_indices.size() - dynamic_indices.size(), new_dynamic_indices.size());
  }

  std::vector<int> NeighborAwareRegionGrowing() {
    // check current grid is valid
    std::vector<int> extra_dynamic_indices;

    int num_dynamic_grid = 0;
    int num_selected_expand_grid = 0;

    for (auto& grid : grid_map_) {
      const GridKey& self_key = grid.first;
      auto& grid_data = grid.second;
      if (!grid_data.is_valid) {
        continue;
      }
      // current grid must be dynamic
      if (!IsGridDynamic(grid_data)) {
        continue;
      }

      // ROS_WARN("====================================== >>>>>>>>>");
      // ROS_WARN("debug grid: nonground %d, dy %d",
      //          grid_data.nonground_indices.size(),
      //          grid_data.dynamic_indices.size());
      // ROS_WARN("====================================== >>>>>>>>>");

      num_dynamic_grid++;
      // neighbor grids must be expandable
      std::vector<Grid*> neighbor_grids;
      if (!getExpandableNeighboringGrids(self_key, neighbor_grids)) {
        continue;
      }

      num_selected_expand_grid ++;
      std::vector<int> new_dynamic_indices;


      this->PerformRegionGrowing(grid_data, neighbor_grids, cloud_,
                                 new_dynamic_indices);
      extra_dynamic_indices.insert(extra_dynamic_indices.end(),
                                   new_dynamic_indices.begin(),
                                   new_dynamic_indices.end());                                 
    }

    // ROS_WARN("num_dynamic_grid: %d, num_selected_expand_grid: %d", num_dynamic_grid, num_selected_expand_grid);
    return extra_dynamic_indices;
  }
};

/////////////////////////////////voxel/////////////////////////////////////

// 定义一个3D向量作为哈希表的键
typedef Eigen::Vector3i VoxelKey;
struct Voxel {
  VoxelKey self_key_;
  std::vector<int> indices;
  bool is_valid_ = false;
  // cv::Mat tmp_range_image;
};



// 自定义哈希函数
struct VoxelHash {
  std::size_t operator()(const VoxelKey& k) const {
    return std::hash<int>()(k[0]) ^ std::hash<int>()(k[1]) << 1 ^
           std::hash<int>()(k[2]) << 2;
  }
};

struct VoxelMap {
  std::unordered_map<VoxelKey, Voxel, VoxelHash> voxel_map_;
  std::unordered_map<int, VoxelKey> point2voxel_;
  pcl::PointCloud<PointType>::ConstPtr cloud_;
  float voxel_size_ = 0.8;
  int num_valid_voxels_ = 0;
  int num_points_total_ = 0;


  VoxelMap(pcl::PointCloud<PointType>::ConstPtr cloud) : cloud_(cloud) {}

  void insertPointIntoVoxelMap(const PointType& point, const int& index) {
    // 计算体素键
    VoxelKey key;
    key[0] = static_cast<int>(floor(point.x / voxel_size_));
    key[1] = static_cast<int>(floor(point.y / voxel_size_));
    key[2] = static_cast<int>(floor(point.z / voxel_size_));

    if (voxel_map_.find(key) == voxel_map_.end()) {
      // 如果哈希表中没有这个体素，创建一个新的体素
      Voxel voxel;
      voxel.self_key_ = key;
      voxel.is_valid_ = true;
      voxel_map_[key] = voxel;
      num_valid_voxels_++;
    }
    num_points_total_++;
    // 插入点索引到哈希表
    // 假设 `index` 是点的唯一索引
    voxel_map_[key].indices.push_back(index);
    point2voxel_[index] = key;
  }

  void GetNeighborVoxels() {
    // if near center, not use neighbor?

  }

  void Project2RangeImage() {

  }

  void PostProcessNormal() {

  }

  bool ComputeNormalFromRangeImage(int target_point_index, Eigen::Vector3d& normal) {
    // get current voxel

    // check point's index on the range image

    // check 5*5, if point not enough, return false
    // if most points nearer than current, maybe it's in dynamic points, return
    // false
    //

    // post process normal
    // normal which points nearly up is ok, but near to downward is not ok
    return true;
  }


};

// typedef std::unordered_map<VoxelKey, Voxel, VoxelHash> VoxelMap;



}  // namespace kumo