#ifndef ASTAR_H
#define ASTAR_H

#include <Eigen/Eigen>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include "SkeletonFinder/backward.hpp"
#include "SkeletonFinder/data_type_3D.h"

namespace a_star {
struct Node;
typedef Node* NodePtr;

struct Node {
  Eigen::Vector3d coord;
  double g_score, f_score;
  bool expanded;
  NodePtr parent;
  vector<NodePtr> connected_node;
  vector<Eigen::Vector3d> connected_node_pos;

  Node(Eigen::Vector3d coord_, vector<Eigen::Vector3d> connected_node_pos_) {
    coord = coord_;
    connected_node_pos = connected_node_pos_;

    // Set default values;
    g_score = -1;
    f_score = -1;
    expanded = false;
    parent = NULL;
  }
};

struct NodeComparator {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class AStar {
 public:
  // Parameter
  double lambda_heu_;

  AStar();
  ~AStar();
  enum { REACH_END = 1, NO_PATH = 2 };

  void init(vector<NodePtr> nodes);
  static double pathLength(const vector<Eigen::Vector3d>& path);
  int search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt);

  std::vector<Eigen::Vector3d> getPath();
  //void visNodes();
  //void visConnections();

 private:
  // parameter
  double tie_breaker_;

  // main data structure
  vector<NodePtr> node_pool;
  std::vector<Eigen::Vector3d> path_nodes;

  void backtrack(const NodePtr& end_node, const Eigen::Vector3d& end);

  double getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  double getEuclDis(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2);
  NodePtr getNodeFromPos(Eigen::Vector3d pos);
  bool isSamePos(Eigen::Vector3d p1, Eigen::Vector3d p2);
  bool isEndNode(NodePtr node, Eigen::Vector3d end);
  //void visParents();
};
}  // namespace a_star

#endif  // ASTAR