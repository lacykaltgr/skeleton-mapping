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
  string file_dir = filename.substr(0, filename.find_last_of("/\\"));
  config_file_name = argv[2];

  YAML::Node config = YAML::LoadFile(config_file_name);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDReader reader;
  reader.read(filename, *cloud);

  cout << "Map successfully loaded..." << endl;
  cout << "Size of map: " << (*cloud).points.size() << endl;

  SkeletonFinder skeleton_finder_3D(config);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  skeleton_finder_3D.run_processing(cloud_xyz);
  //skeleton_finder_3D.run_postprocessing();

      // save valid nodes to file
  skeleton_finder_3D.saveNodes(file_dir + "/nodes.pcd");

  // save connections to file
  skeleton_finder_3D.saveConnections(file_dir + "/connections.txt");

  // save adjacency matrix to file
  skeleton_finder_3D.saveAdjMatrix(file_dir + "/adjacency_matrix.txt");
}