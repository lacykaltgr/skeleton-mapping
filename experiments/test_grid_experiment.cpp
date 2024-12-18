#include "SkeletonFinder/skeleton_finder_3D.h"
#include "SkeletonFinder/experiments.h"
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
string obj_filename;

int main(int argc, char **argv) {
    pcd_filename = argv[1];
    config_filename = argv[2];
    obj_filename = argv[3];
    double leaf_size = atof(argv[4]);
    YAML::Node config = YAML::LoadFile(config_filename);

    string experiment_name = "grid_" + to_string(leaf_size);
    SkeletonFinderExperiment experiment(experiment_name, config["robot_type"].as<double>(), pcd_filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDReader reader;
    reader.read(pcd_filename, *cloud);

    cout << "Map successfully loaded..." << endl;
    cout << "Size of map: " << (*cloud).points.size() << endl;

    
    SkeletonFinder skeleton_finder_3D(config);
    skeleton_finder_3D.run_processing_grid(cloud, leaf_size);   
    skeleton_finder_3D.run_postprocessing();

    experiment.run_experiments(skeleton_finder_3D, obj_filename);
}