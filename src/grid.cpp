#include "SkeletonFinder/skeleton_finder_3D.h"
#include <algorithm>
#include <chrono>
#include <fstream>

using namespace Eigen;
using namespace std;


void SkeletonFinder::run_processing_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size, double base_height) {

    // find bounding box
    findBoundingBox(map_pcl);

    // create grid
    vector<vector<NodePtr>> grid;
    for (double x = _x_min; x < _x_max; x += cell_size) {
        vector<NodePtr> row;
        for (double y = _y_min; y < _y_max; y += cell_size) {
            Eigen::Vector3d point(x, y, base_height);
            NodePtr node = new Node(point, NULL);
            initNode(node);
            row.push_back(node);
        }
        grid.push_back(row);
    }

    int nX = grid.size();
    int nY = grid[0].size();

    // connect nodes
    for (int i = 0; i < nX; i++) {
        for (int j = 0; j < nY; j++) {
            NodePtr node = grid[i][j];
            if (i > 0) {
                node->neighbors.push_back(grid[i - 1][j]);
            }
            if (i < nX - 1) {
                node->neighbors.push_back(grid[i + 1][j]);
            }
            if (j > 0) {
                node->neighbors.push_back(grid[i][j - 1]);
            }
            if (j < nY - 1) {
                node->neighbors.push_back(grid[i][j + 1]);
            }
        }
    }


    // run processing
}