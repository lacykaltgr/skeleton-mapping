#include "SkeletonFinder/skeleton_finder_3D.h"

using namespace Eigen;
using namespace std;


vector<NodeNearestNeighbors> SkeletonFinder::run_nearestnodes() {
  vector<NodeNearestNeighbors> nearest_nodes;
  // TODO: should we use center_NodeList (exclude gates)
  int counter = 0;
  kdtreeForNodes.setInputCloud(nodes_pcl);
  for (NodePtr node: NodeList) {
    NodeNearestNeighbors nearest_node;
    nearest_node.index = counter++;
    nearest_node.coord = node->coord;
    nearest_node.nearest_nodes = findNearestNodes(node);
    nearest_nodes.push_back(nearest_node);
  }
  return nearest_nodes;
}


vector<int> SkeletonFinder::findNearestNodes(NodePtr node) {
  vector<int> nearest_nodes;
  pcl::PointXYZ pcl_start(node->coord(0), node->coord(1), node->coord(2));
  pointIdxRadiusSearchForNodes.clear();
  pointRadiusSquaredDistanceForNodes.clear();
  kdtreeForNodes.nearestKSearch(pcl_start, 5, pointIdxRadiusSearchForNodes,
                                pointRadiusSquaredDistanceForNodes);
  for (std::size_t i = 0; i < pointIdxRadiusSearchForNodes.size(); ++i) {
    int node_index = pointIdxRadiusSearchForNodes[i];
    nearest_nodes.push_back(node_index);
  }
  return nearest_nodes;
}