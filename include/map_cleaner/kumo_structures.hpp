#pragma once
// strange data structures
// #include <opencv2/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <grid_map_ros/grid_map_ros.hpp>
#include <map_cleaner/utils.hpp>
#include <unordered_map>
#include <vector>

namespace kumo {
/////////////////////////////////cell/////////////////////////////////////
struct Grid {
  float x_center = std::numeric_limits<float>::quiet_NaN(), y_center = std::numeric_limits<float>::quiet_NaN();
  std::vector<int> ground_indices;
  std::vector<int> nonground_indices;
  int num_points_total = 0;
  int num_ground = 0;
  int num_dynamic = 0;
  float z_terrain = 0.0;
  bool is_valid = false;
};
// 定义一个2D向量作为哈希表的键
typedef Eigen::Vector2i GridKey;

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
  

  // GridMap(const float& grid_size) : grid_size_(grid_size) {}
  GridMap() {}

  void InitGridTerrainHeights(const grid_map::GridMap& grid_terrain, std::string input_layer_name) {
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
      if (!grid_terrain.getIndex(grid_map::Position(grid_data.x_center, grid_data.y_center), idx)) {
        ROS_ERROR( "grid_terrain.getIndex failed, position: (%f, %f)", grid_data.x_center, grid_data.y_center);
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
    return;
    for (auto& grid : grid_map_) {
      auto& grid_data = grid.second;
      if (!grid_data.is_valid) {
        continue;
      }
      const int num_ground = grid_data.num_ground;
      ROS_WARN("grid pos: %f,%f, num_ground: %d", grid_data.x_center, grid_data.y_center, num_ground);
    }
  }

  inline void IncreaseDynamicNums(int index) {
    auto& grid = grid_map_[point2grid_[index]];
    grid.num_dynamic += 1;
  }

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

      if (num_nondynamic == 0 || 1.0 * num_dynamic / num_nonground >
              thr_expand_dynamic_ratio_ &&
          1.0 * num_ground / num_nondynamic > thr_expand_ground_ratio_) {
        dynamic_points.insert(dynamic_points.end(), grid_data.nonground_indices.begin(),
                              grid_data.nonground_indices.end());
        DEBUG_num_expand_grid++;
      }
    }
    ROS_WARN("expand dynamic grid num: %d", DEBUG_num_expand_grid);
    return dynamic_points;
  }
};

/////////////////////////////////voxel/////////////////////////////////////

struct Voxel {
  std::vector<int> indices;
  // cv::Mat tmp_range_image;
};

// 定义一个3D向量作为哈希表的键
typedef Eigen::Vector3i VoxelKey;

// 自定义哈希函数
struct VoxelHash {
  std::size_t operator()(const VoxelKey& k) const {
    return std::hash<int>()(k[0]) ^ std::hash<int>()(k[1]) << 1 ^
           std::hash<int>()(k[2]) << 2;
  }
};

typedef std::unordered_map<VoxelKey, Voxel, VoxelHash> VoxelMap;

void insertPointIntoVoxelMap(const PointType& point, const int& index,
                             const float& voxel_size, VoxelMap& voxel_map,
                             std::unordered_map<int, VoxelKey>& point2voxel) {
  // 计算体素键
  VoxelKey key;
  key[0] = static_cast<int>(floor(point.x / voxel_size));
  key[1] = static_cast<int>(floor(point.y / voxel_size));
  key[2] = static_cast<int>(floor(point.z / voxel_size));

  // 插入点索引到哈希表
  // 假设 `index` 是点的唯一索引
  voxel_map[key].indices.push_back(index);
  point2voxel[index] = key;
}

}  // namespace kumo

// TODO
// point2cell
// point2voxel
//
// std::unordered_set<VoxelKey, VoxelHash> uniqueKeys(selectedVoxelKeys.begin(),
// selectedVoxelKeys.end()); std::vector<VoxelKey>
// uniqueVoxelKeys(uniqueKeys.begin(), uniqueKeys.end());
// 在这种情况下，uniqueVoxelKeys 将包含无重复的 VoxelKey 列表。