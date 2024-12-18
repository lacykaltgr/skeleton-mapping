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
string out_dir;

// Custom class to store Gaussian parameters

int main(int argc, char **argv) {
  filename = argv[1];
  out_dir = argv[2];
  config_file_name = argv[3];

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

      // save valid nodes to file
  skeleton_finder_3D.saveNodes(out_dir + "/nodes_0.pcd");

  // save connections to file
  skeleton_finder_3D.saveConnections(out_dir + "/connections_0.txt");

  // save adjacency matrix to file
  skeleton_finder_3D.saveAdjMatrix(out_dir + "/adjacency_matrix_0.txt");

  skeleton_finder_3D.run_postprocessing();

    // save valid nodes to file
  skeleton_finder_3D.saveNodes(out_dir + "/nodes.pcd");

  // save connections to file
  skeleton_finder_3D.saveConnections(out_dir + "/connections.txt");

  // save adjacency matrix to file
  skeleton_finder_3D.saveAdjMatrix(out_dir + "/adjacency_matrix.txt");
}