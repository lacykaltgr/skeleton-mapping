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
    YAML::Node config = YAML::LoadFile(config_filename);

    string experiment_name = "Skeleton Finder";
    SkeletonFinderExperiment experiment(experiment_name, config["robot_type"].as<double>(), pcd_filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDReader reader;
    reader.read(pcd_filename, *cloud);

    cout << "Map successfully loaded..." << endl;
    cout << "Size of map: " << (*cloud).points.size() << endl;

    
    SkeletonFinder skeleton_finder_3D(config);
    skeleton_finder_3D.run_processing(cloud);
    skeleton_finder_3D.fillNodesPcl(-1, -1);

    for (NodePtr node: skeleton_finder_3D.getNodes()) {
        if (node->rollbacked) {
            continue;
        }
        for (NodePtr ngbr: node->connected_Node_ptr) {
            if (ngbr == node) {
                continue;
            }
            if (find(skeleton_finder_3D.getNodes().begin(), skeleton_finder_3D.getNodes().end(), ngbr) == skeleton_finder_3D.getNodes().end()) {
                cout << "Invalid neighbor found!" << endl;
                cout << "Node: " << node->coord << endl;
                cout << "Neighbor: " << ngbr->coord << endl;
            }
        }
    }

    experiment.run_experiments(skeleton_finder_3D, obj_filename);
}