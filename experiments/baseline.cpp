#include "SkeletonFinder/skeleton_finder_3D.h"
#include <algorithm>
#include <chrono>
#include <fstream>

using namespace Eigen;
using namespace std;


void SkeletonFinder::run_processing_grid_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size, double base_height) {

    // find bounding box
    findBoundingBox(map_pcl);

    // create 2D grid
    for (double x = _x_min; x < _x_max; x += cell_size) {
        for (double y = _y_min; y < _y_max; y += cell_size) {
            Eigen::Vector3d point(x, y, base_height);
            double collision_radius = sqrt(2) * cell_size / 2;
            if (collisionCheck(point, collision_radius)) continue;
            NodePtr node = new Node(point, NULL);
            recordNode(node);
        }
    }
}

void SkeletonFinder::run_processing_grid_3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size) {

    // find bounding box
    findBoundingBox(map_pcl);

    // create 3D grid
    for (double x = _x_min; x < _x_max; x += cell_size) {
        for (double y = _y_min; y < _y_max; y += cell_size) {
            for (double z = _z_min; z < _z_max; z += cell_size) {
                Eigen::Vector3d point(x, y, z);
                double collision_radius = sqrt(2) * cell_size / 2;
                if (collisionCheck(point, collision_radius)) continue;
                NodePtr node = new Node(point, NULL);
                recordNode(node);
            }
        }
    }
}


void SkeletonFinder::run_processing_random_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, int num_points, double base_height, double base_radius) {
    // Find the bounding box of the point cloud
    findBoundingBox(raw_map_pcl);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(_x_min, _x_max);
    std::uniform_real_distribution<> dis_y(_y_min, _y_max);

    // Randomly sample points until the required number of valid points is reached
    while (NodeList.size() < num_points) {
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = base_height;
        Eigen::Vector3d point(x, y, z);
        if (collisionCheck(point, base_radius)) continue;
        NodePtr node = new Node(point, NULL);
        recordNode(node);
    }
}



void SkeletonFinder::run_processing_random_3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, int num_points, double base_radius) {
        // Find the bounding box of the point cloud
    findBoundingBox(raw_map_pcl);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(_x_min, _x_max);
    std::uniform_real_distribution<> dis_y(_y_min, _y_max);
    std::uniform_real_distribution<> dis_z(_z_min, _z_max);

    // Randomly sample points until the required number of valid points is reached
    while (NodeList.size() < num_points) {
        double x = dis_x(gen);
        double y = dis_y(gen);
        double z = dis_z(gen);
        Eigen::Vector3d point(x, y, z);
        if (collisionCheck(point, base_radius)) continue;
        NodePtr node = new Node(point, NULL);
        recordNode(node);
    }
}



