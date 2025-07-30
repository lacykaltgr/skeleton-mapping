#include "SkeletonFinder/skeleton_finder_3D.h"
#include <pcl/io/pcd_io.h>

using namespace Eigen;
using namespace std;


/*
Post-processing of 3D skeleton

1. Extend the skeleton graph by finding unexplored areas
  - Find nodes with only 1 connection in the original graph
  - Run the algorithm again initialized from that point, or its neighbor
  - Repeat until bounding box coverage is above a threshold
2. Remove nodes which are above/too close to objects
  - Remove nodes which are above objects
  - Find merging point for objects which are too close 
  - (weighted by the number of connections, more connections -> more important)
3. Draw edges based on the clear path
  - Find valid edges
  - Add edges where the path is clear


IDEA: store for every neighbour from what radius of the base node is the path always clear

*/


void SkeletonFinder::run_postprocessing() {
  // print all node coordinates

  // reinitialize nodes_pcl
  if (_robot_type == 0) {
    fillNodesPcl(_base_height, -1);
  } else {
    fillNodesPcl(-1, -1);
  }
  //cout << "Number of valid nodes: " << NodeList.size() << endl;

  // add edges where the path is clear
  int nEdges = drawEdgesRawWalking(NodeList, _connection_radius);
  cout << "Number of edges added: " << nEdges << endl;
  
  // iteratively remove too close nodes, while there is none
  cout << "Number of nodes before removing too close nodes: " << NodeList.size() << endl;
  int nCloseFoundLast = -1;
  for (int nCloseFound = removeTooCloseNodes(_too_close_threshold); nCloseFound > 0;  nCloseFound = removeTooCloseNodes(_too_close_threshold)) {
    if (nCloseFound == nCloseFoundLast) {
      break;
    }
    nCloseFoundLast = nCloseFound;
  }
  cout << "Number of nodes after removing too close nodes: " << NodeList.size() << endl;


  // fill nodes pcl with valid nodes only
  fillNodesPcl(-1, _base_radius);
  nEdges = drawEdgesRawWalking(NodeList, _connection_radius);
  cout << "Number of edges added after removing close nodes: " << nEdges << endl;

  vector<vector<int>> components = getDisconnectedComponents(NodeList);
  cout << "Number of components: " << components.size() << endl;

  // keep only largest component
  if (components.size() > 0) {
    vector<NodePtr> largestComponent;
    for (int i: components[0]) {
      largestComponent.push_back(NodeList[i]);
    }
    NodeList = largestComponent;
  }
  fillNodesPcl(-1, -1);


  cout << "Number of nodes in the largest component: " << NodeList.size() << endl;

  cout << "Post-processing finished." << endl;
}


void SkeletonFinder::run_postprocess_edges() {
    // print all node coordinates

  // reinitialize nodes_pcl
  if (_robot_type == 0) {
    fillNodesPcl(_base_height, _base_radius);
  } else {
    fillNodesPcl(-1, _base_radius);
  }

  drawEdgesRawWalking(NodeList, _connection_radius);

  vector<vector<int>> components = getDisconnectedComponents(NodeList);
  cout << "Number of components: " << components.size() << endl;

  // keep only largest component
  if (components.size() > 0) {
    vector<NodePtr> largestComponent;
    for (int i: components[0]) {
      largestComponent.push_back(NodeList[i]);
    }
    NodeList = largestComponent;
  }
  fillNodesPcl(-1, -1);


  cout << "Number of nodes in the largest component: " << NodeList.size() << endl;

  cout << "Post-processing finished." << endl;
}


void SkeletonFinder::fillNodesPcl(double base_height=-1, double base_radius=-1) {
  bool checkForCollision = base_radius > 0;
  vector<NodePtr> validNodeList;
  int index = 0;
  nodes_pcl->clear();
  for (NodePtr node: NodeList) {

    if (checkForCollision) {
      node->connected_Node_ptr.clear();
    } else {
      vector<NodePtr> invalid_ngbrs;
      for (NodePtr ngbr: node->connected_Node_ptr)
        if (find(NodeList.begin(), NodeList.end(), ngbr) == NodeList.end())
          invalid_ngbrs.push_back(ngbr);
      for (NodePtr ngbr: invalid_ngbrs)
        node->connected_Node_ptr.erase(
          remove(node->connected_Node_ptr.begin(), node->connected_Node_ptr.end(), ngbr), 
          node->connected_Node_ptr.end()
        );
    }

    if (base_height > 0) {
      node->coord(2) = base_height;
    }

    if (node->rollbacked) {
      continue;
    }

    if (checkForCollision && collisionCheck(node->coord, base_radius)) {
      continue;
    }

    node->index = index++;
    validNodeList.push_back(node);
    nodes_pcl->points.push_back(
      pcl::PointXYZ(node->coord(0), node->coord(1), node->coord(2))
    );
  }

  NodeList = validNodeList;
  assert(nodes_pcl->points.size() == NodeList.size());
}

int SkeletonFinder::drawEdgesRawWalking(vector<NodePtr>& validNodeList, double connectionRadius) {
  int counter = 0;
  kdtreeForNodes.setInputCloud(nodes_pcl);

  for (NodePtr node : validNodeList) {
    // radius search on valid nodes
    vector<NodePtr> nearest_nodes = closestNodes(node, connectionRadius, validNodeList);

    // add edges where the path is clear
    for (NodePtr nearest_node : nearest_nodes) {
      if (nearest_node == node)
        continue;

      auto finder1 = find(validNodeList.begin(), validNodeList.end(), nearest_node);
      if (finder1 == validNodeList.end())
        continue;
      
      // check if the edge already exists
      auto finder = find(node->connected_Node_ptr.begin(), node->connected_Node_ptr.end(), nearest_node);
      if (finder != node->connected_Node_ptr.end())
        continue;
      
      auto start = Eigen::Vector3d(node->coord(0), node->coord(1), node->coord(2));
      auto target = Eigen::Vector3d(nearest_node->coord(0), nearest_node->coord(1), nearest_node->coord(2));

      if (checkPathClear(start, target)) {
        node->connected_Node_ptr.push_back(nearest_node);
        nearest_node->connected_Node_ptr.push_back(node);
        counter++;
      }
    }
  }

  return counter;
}




void SkeletonFinder::mergeNodes(NodePtr node1, NodePtr node2, vector<NodePtr>& validNodeList, double node1_w, double node2_w) {
  // create new node weighted by the weights
  Eigen::Vector3d new_coord = (node1->coord * node1_w + node2->coord * node2_w) / (node1_w + node2_w);
  node1->coord = new_coord;

  // add all edges from one node to another
  // does not check for collision validity
  killOtherNode(node1, node2, validNodeList);
}

// add all edges from one node to another
// does not check for collision validity
void SkeletonFinder::killOtherNode(NodePtr node_to_keep, NodePtr node_to_remove, vector<NodePtr>& validNodeList) {
  // loop though the connected nodes of node_to_remove
  // add the connected nodes to node_to_keep
  for (NodePtr connected_node : node_to_remove->connected_Node_ptr) {
    // skip if the connected node is the node_to_keep
    if (connected_node == node_to_keep)
      continue;

    // skip if connected node is connected to node_to_keep
    auto finder = find(node_to_keep->connected_Node_ptr.begin(), node_to_keep->connected_Node_ptr.end(), connected_node);
    if (finder != node_to_keep->connected_Node_ptr.end())
      continue;

    // add the connected node to node_to_keep
    node_to_keep->connected_Node_ptr.push_back(connected_node);
    connected_node->connected_Node_ptr.push_back(node_to_keep);
    connected_node->connected_Node_ptr.erase(
      remove(connected_node->connected_Node_ptr.begin(), connected_node->connected_Node_ptr.end(), node_to_remove),
      connected_node->connected_Node_ptr.end()
    );
  }

  // remove node_to_remove from validNodeList
  NodeList.erase(
    remove(validNodeList.begin(), validNodeList.end(), node_to_remove),
    validNodeList.end()
  );
}


vector<NodePtr> SkeletonFinder::closestNodes(NodePtr node, double maxDistance, vector<NodePtr>& validNodeList) {
    vector<NodePtr> nearest_nodes;

    pcl::PointXYZ pcl_start(node->coord(0), node->coord(1), node->coord(2));

    pointIdxRadiusSearchForNodes.clear();
    pointRadiusSquaredDistanceForNodes.clear();
    kdtreeForNodes.radiusSearch(pcl_start, maxDistance, pointIdxRadiusSearchForNodes,
                                  pointRadiusSquaredDistanceForNodes);

    for (std::size_t i = 0; i < pointIdxRadiusSearchForNodes.size(); ++i) {
      int node_index = pointIdxRadiusSearchForNodes[i];
      nearest_nodes.push_back(validNodeList[node_index]); 
    }
    return nearest_nodes;
}


int SkeletonFinder::removeTooCloseNodes(double tooCloseThreshold) {
  cout << "Removing too close nodes..." << endl;
  kdtreeForNodes.setInputCloud(nodes_pcl);

  vector<TooCloseCandidate> tooCloseCandidates;
  for (NodePtr node: NodeList) {
    
    vector<NodePtr> nodes_too_close = closestNodes(node, tooCloseThreshold, NodeList);

    for (NodePtr nearest_node: nodes_too_close) {
      if (nearest_node == node)
        continue;
      bool added = false;
      for (TooCloseCandidate candidate: tooCloseCandidates) {
        if (candidate.node1 == nearest_node && candidate.node2 == node) {
          added = true;
          break;
        }
      }
      if (added)
        continue;
      TooCloseCandidate candidate;
      candidate.node1 = node;
      candidate.node2 = nearest_node;
      candidate.distance = getDis(node->coord, nearest_node->coord);
      tooCloseCandidates.push_back(candidate);
    }
  }

  // sort tooCloseCandidates by distance
  sort(tooCloseCandidates.begin(), tooCloseCandidates.end(), 
    [](const TooCloseCandidate &a, const TooCloseCandidate &b) -> bool {
      return a.distance < b.distance;
    }
  );
  // loop through tooCloseCandidates
  for (TooCloseCandidate candidate : tooCloseCandidates) {
    NodePtr node1 = candidate.node1;
    NodePtr node2 = candidate.node2;

    auto finder1 = find(NodeList.begin(), NodeList.end(), node1);
    auto finder2 = find(NodeList.begin(), NodeList.end(), node2);

    if (finder1 == NodeList.end() || finder2 == NodeList.end())
      continue;

    // IDEA FOR EXTENSION?? 
    // draw line segment between node1 and node2
    // extend the line segment to the left and right
    // check if the path is clear (not close to any other nodes, on either side)
    // move each as mush as it can
    // place them where optimal
    // if it cant, then find somewhere between them to merge
    // if it cant, then remove one of them

    bool node1_canreplace_node2 = canBeReplacedBy(node1, node2);
    bool node2_canreplace_node1 = canBeReplacedBy(node2, node1);

    if (node1_canreplace_node2 && node2_canreplace_node1) {
      // merge the nodes
      // connection size is not the best weight here, but it is a start
      double node1_w = node1->connected_Node_ptr.size();
      double node2_w = node2->connected_Node_ptr.size();

      if (node1_w == 0 && node2_w == 0) {
        node1_w = 1;
        node2_w = 1;
      }

      mergeNodes(node1, node2, NodeList, node1_w, node2_w);
    } else if (node1_canreplace_node2) {
      killOtherNode(node1, node2, NodeList);
    } else if (node2_canreplace_node1) {
      killOtherNode(node2, node1, NodeList);
    } else { //NOTE:: added this to always remove close nodes
      //killOtherNode(node1, node2, NodeList);
    }
  }
  fillNodesPcl(-1, -1);
  return tooCloseCandidates.size();
}



bool SkeletonFinder::canBeReplacedBy(NodePtr node_to_keep, NodePtr node_to_remove) {
  if (collisionCheck(node_to_remove->coord, _base_radius))
    return true;
  if (node_to_remove->connected_Node_ptr.size() == 1 && node_to_remove->connected_Node_ptr[0] == node_to_keep)
    return true;
  for (NodePtr connected_node : node_to_remove->connected_Node_ptr) {
    // skip if the connected node is the node_to_keep
    if (connected_node == node_to_keep)
      continue;

    // skip if connected node is connected to node_to_keep
    auto finder = find(node_to_keep->connected_Node_ptr.begin(), node_to_keep->connected_Node_ptr.end(), connected_node);
    if (finder != node_to_keep->connected_Node_ptr.end())
      continue;

    if (!checkPathClear(node_to_keep->coord, connected_node->coord))
      return false;
  }
  return true;
}




