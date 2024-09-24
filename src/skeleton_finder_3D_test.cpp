#include "SkeletonFinder/skeleton_finder_3D.h"
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/time.h>
#include <time.h>
#include <pcl/search/impl/kdtree.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <yaml-cpp/yaml.h>


using namespace std;
string config_file_name;
string filename;

// Custom class to store Gaussian parameters

int main(int argc, char **argv) {
  filename = argv[1];
  if (argc < 3) {
    config_file_name = "/app/graph/3D_Sparse_Skeleton/3d_sparse_skeleton/polygon_generation/config.yaml";
  } else {
    config_file_name = argv[2];
  }

  YAML::Node config = YAML::LoadFile(config_file_name);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PCDReader reader;
  reader.read(filename, *cloud);

  cout << "Map successfully loaded..." << endl;
  cout << "Size of map: " << (*cloud).points.size() << endl;

  SkeletonFinder skeleton_finder_3D(config);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  skeleton_finder_3D.run_processing(*cloud_xyz);

  double path_start_x = config["path_start"]["x"].as<double>();
  double path_start_y = config["path_start"]["y"].as<double>();
  double path_start_z = config["path_start"]["z"].as<double>();
  double path_target_x = config["path_target"]["x"].as<double>();
  double path_target_y = config["path_target"]["y"].as<double>();
  double path_target_z = config["path_target"]["z"].as<double>();
  skeleton_finder_3D.run_findpath(
    path_start_x, path_start_y, path_start_z,
    path_target_x, path_target_y, path_target_z
  );
}