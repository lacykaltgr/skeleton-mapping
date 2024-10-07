#include "SkeletonFinder/skeleton_finder_3D.h"

// includes for pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

using namespace Eigen;
using namespace std;


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