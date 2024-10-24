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
string config_filename;
string pcd_filename;
string obj_filename
string output_filename


int main(int argc, char **argv) {
    pcd_filename = argv[1];
    config_filename = argv[2];
    obj_filename = argv[3];
    leaf_size = atof(argv[4]);

    cout << "Running Experiment: Grid 3D" << "Leaf size: " << leaf_size << endl;
    cout << "--------------------------------" << endl;


    pcd_name = pcd_filename.substr(pcd_filename.find_last_of("/\\") + 1);
    output_filename = "output/experiment_grid_" + leaf_size + pcd_name + ".json";

    YAML::Node config = YAML::LoadFile(config_filename);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PCDReader reader;
    reader.read(filename, *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud, *cloud_xyz);

    cout << "Map successfully loaded..." << endl;
    cout << "Size of map: " << (*cloud).points.size() << endl;

    
    SkeletonFinder skeleton_finder_3D(config);
    skeleton_finder_3D.run_processing_grid_3d(
        cloud_xyz,
        leaf_size,
    );   
    skeleton_finder_3D.run_postprocessing(
        config["base_height"].as<double>(), 
        config["connection_radius"].as<double>(), 
        config["too_close_radius"].as<double>()
    );
    skeleton_finder.run_experiments()
  
}