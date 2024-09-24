#include "SkeletonFinder/A_star.h"
#include <algorithm>

// using namespace Eigen;
// using namespace std;

// namespace backward {
// backward::SignalHandling sh;
// }

namespace a_star {
AStar::AStar() {}

AStar::~AStar() {}

double AStar::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < path.size() - 1; ++i) length += (path[i + 1] - path[i]).norm();
  return length;
}

void AStar::init(vector<NodePtr> nodes) {
  tie_breaker_ = 1.0 + 1.0 / 1000;
  for (NodePtr node : nodes) {
    node_pool.push_back(node);
  }

  for (NodePtr node : nodes) {
    for (Eigen::Vector3d con_node_pos : node->connected_node_pos) {
      NodePtr con_node = getNodeFromPos(con_node_pos);
      if (con_node == NULL) {
        std:cout << "Can't find Node at given position(%f, %f, %f)!" << con_node_pos(0) << con_node_pos(1) << con_node_pos(2) << endl;
        continue;
      }
      node->connected_node.push_back(con_node);
    }
  }
}

int AStar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
  NodePtr start = getNodeFromPos(start_pt);
  start->g_score = 0;
  start->f_score = getEuclHeu(start_pt, end_pt);
  queue.push(start);

  NodePtr cur_node;
  while (!queue.empty()) {
    cur_node = queue.top();
    // ROS_WARN("Expanding node (%f, %f, %f), g = %f, f = %f", cur_node->coord(0), cur_node->coord(1),
    //          cur_node->coord(2), cur_node->g_score, cur_node->f_score);
    if (isEndNode(cur_node, end_pt)) {
      backtrack(cur_node, end_pt);
      return REACH_END;
    }

    queue.pop();
    cur_node->expanded = true;

    // ROS_INFO("Number of connected nodes: %d", cur_node->connected_node.size());
    for (NodePtr nbhd_node : cur_node->connected_node) {
      if (nbhd_node->expanded) continue;

      double nbhd_g = cur_node->g_score + getEuclDis(cur_node->coord, nbhd_node->coord);
      if (nbhd_node->g_score < 0) {
        nbhd_node->parent = cur_node;
        // ROS_INFO("Change parent of node (%f, %f, %f) to current node.", nbhd_node->coord(0),
        //          nbhd_node->coord(1), nbhd_node->coord(2));
        nbhd_node->g_score = nbhd_g;
        nbhd_node->f_score = nbhd_node->g_score + getEuclHeu(nbhd_node->coord, end_pt);
        queue.push(nbhd_node);
      } else if (nbhd_node->g_score > nbhd_g) {
        nbhd_node->g_score = nbhd_g;
        nbhd_node->parent = cur_node;
        // ROS_INFO("Change parent of node (%f, %f, %f) to current node.", nbhd_node->coord(0),
        //          nbhd_node->coord(1), nbhd_node->coord(2));
      }
    }

    // ROS_WARN("-------------------------------------");
    // visParents();
    // getchar();
  }

  return NO_PATH;
}

std::vector<Eigen::Vector3d> AStar::getPath() { return path_nodes; }

void AStar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
  path_nodes.push_back(end);
  path_nodes.push_back(end_node->coord);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes.push_back(cur_node->coord);
  }
  reverse(path_nodes.begin(), path_nodes.end());
}

double AStar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

double AStar::getEuclDis(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return (x2 - x1).norm();
}

NodePtr AStar::getNodeFromPos(Eigen::Vector3d pos) {
  for (NodePtr node : node_pool) {
    if (isSamePos(node->coord, pos)) return node;
  }
  return NULL;
}

bool AStar::isSamePos(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  return fabs(p1(0) - p2(0)) < 1e-3 && fabs(p1(1) - p2(1)) < 1e-3 && fabs(p1(2) - p2(2)) < 1e-3;
}

bool AStar::isEndNode(NodePtr node, Eigen::Vector3d end) { return isSamePos(node->coord, end); }


}  // namespace a_star
