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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDReader reader;
  reader.read(filename, *cloud);

  cout << "Map successfully loaded..." << endl;
  cout << "Size of map: " << (*cloud).points.size() << endl;

  SkeletonFinder skeleton_finder_3D(config);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  double downsample = config["downsample"].as<double>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled;
  if (downsample > 0) {
    cout << "Downsampling map..." << endl;
    downsampled = skeleton_finder_3D.downsample(cloud_xyz, downsample);
    cout << "Downsampled map size: " << downsampled->points.size() << endl;
  } else {
    downsampled = cloud_xyz;
  }

  skeleton_finder_3D.run_processing(downsampled);
  skeleton_finder_3D.run_postprocessing(0.5, 5, 0.5);
}