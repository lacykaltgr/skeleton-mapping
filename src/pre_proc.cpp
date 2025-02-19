#include "SkeletonFinder/skeleton_finder_3D.h"

// includes for pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

using namespace Eigen;
using namespace std;


void SkeletonFinder::findBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr map) {
  bool findXmin = _x_min == -1;
  bool findXmax = _x_max == -1;
  bool findYmin = _y_min == -1;
  bool findYmax = _y_max == -1;
  bool findZmin = _z_min == -1;
  bool findZmax = _z_max == -1;

  if (!findXmin && !findXmax && !findYmin && !findYmax && !findZmin && !findZmax) {
    return;
  }

  _x_min = findXmin ? std::numeric_limits<double>::max() : _x_min;
  _x_max = findXmax ? -std::numeric_limits<double>::min() : _x_max;
  _y_min = findYmin ? std::numeric_limits<double>::max() : _y_min;
  _y_max = findYmax ? -std::numeric_limits<double>::min() : _y_max;
  _z_min = findZmin ? std::numeric_limits<double>::max() : _z_min;
  _z_max = findZmax ? -std::numeric_limits<double>::min() : _z_max;

  for (size_t i = 0; i < map->points.size(); i++) {
    if (findXmin && map->points[i].x < _x_min) {
      _x_min = map->points[i].x;
    }
    if (findXmax && map->points[i].x > _x_max) {
      _x_max = map->points[i].x;
    }
    if (findYmin && map->points[i].y < _y_min) {
      _y_min = map->points[i].y;
    }
    if (findYmax && map->points[i].y > _y_max) {
      _y_max = map->points[i].y;
    }
    if (findZmin && map->points[i].z < _z_min) {
      _z_min = map->points[i].z;
    }
    if (findZmax && map->points[i].z > _z_max) {
      _z_max = map->points[i].z;
    }
  }
}


void SkeletonFinder::addBbxToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map) {
  //double x_length = _x_max - _x_min;
  //double y_length = _y_max - _y_min;
  // double z_length = _z_max - _z_min;
  //int x_num = ceil(x_length / _resolution) + 1;
  //int y_num = ceil(y_length / _resolution) + 1;
  // int z_num = ceil(z_length / _resolution) + 1;

  /*
  // Add ceiling and/or floor to the map

  if (_is_simulation) { // Add ceiling and floor to search map
    for (int i = 0; i < x_num; i++) {
      for (int j = 0; j < y_num; j++) {
        map->points.push_back(pcl::PointXYZ(_x_min + _resolution * i,
                                           _y_min + _resolution * j, _z_min));
        map->points.push_back(pcl::PointXYZ(_x_min + _resolution * i,
                                           _y_min + _resolution * j, _z_max));
      }
    }
  } else { // Add only ceiling to search map
    for (int i = 0; i < x_num; i++) {
      for (int j = 0; j < y_num; j++) {
        map->points.push_back(pcl::PointXYZ(_x_min + _resolution * i,
                                           _y_min + _resolution * j, _z_max));
      }
    }
  }
  */
}


void SkeletonFinder::addInitialFrontier(FrontierPtr frontier) {
  frontier->index = loop_candidate_frontiers.size();
  loop_candidate_frontiers.push_back(frontier);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr SkeletonFinder::downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double leaf) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf, leaf, leaf);
  sor.filter(*cloud_filtered);
  return cloud_filtered;
}

void SkeletonFinder::setParam(
  double x_min, double x_max, double y_min, double y_max, double z_min, double z_max,
  double start_x, double start_y, double start_z,
  double frontier_creation_threshold, double frontier_jump_threshold, double frontier_split_threshold,
  int min_flowback_creation_threshold, double min_flowback_creation_radius_threshold,
  double min_node_radius, double search_margin, double max_ray_length,
  double max_expansion_ray_length, double max_height_diff, int sampling_density,
  int max_facets_grouped, double resolution,  bool bad_loop,
  double base_height, double base_radius, double connection_radius, double too_close_threshold,
  int robot_type, double raywalking_max_height_diff, double min_hit_ratio, bool exploration_mode
) {
  _x_min = x_min;
  _x_max = x_max;
  _y_min = y_min;
  _y_max = y_max;
  _z_min = z_min;
  _z_max = z_max;
  _start_x = start_x;
  _start_y = start_y;
  _start_z = start_z;

  _frontier_creation_threshold = frontier_creation_threshold;
  _frontier_jump_threshold = frontier_jump_threshold;
  _frontier_split_threshold = frontier_split_threshold;
  _min_flowback_creation_threshold = min_flowback_creation_threshold;
  _min_flowback_creation_radius_threshold = min_flowback_creation_radius_threshold;

  _min_node_radius = min_node_radius;
  _search_margin = search_margin;
  _max_ray_length = max_ray_length;
  _max_expansion_ray_length = max_expansion_ray_length;
  _max_height_diff = max_height_diff;
  _sampling_density = sampling_density;
  _max_facets_grouped = max_facets_grouped;
  _resolution = resolution;

  _bad_loop = bad_loop;

  _base_height = base_height;
  _base_radius = base_radius;
  _connection_radius = connection_radius;
  _too_close_threshold = too_close_threshold;
  _robot_type = robot_type;
  _raywalking_max_height_diff = raywalking_max_height_diff;
  _min_hit_ratio = min_hit_ratio;
  _exploration_mode = exploration_mode;
}
