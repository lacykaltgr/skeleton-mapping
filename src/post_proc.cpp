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
4. Spectral clustering



IDEA: store for every neighbour from what radius of the base node is the path always clear

*/

void SkeletonFinder::fillNodesPcl(vector<NodePtr> nodeList, double base_radius=-1) {
  bool checkForCollision = base_radius > 0;
  vector<NodePtr> invalidNodeList;
  nodes_pcl->clear();
  for (NodePtr node: nodeList) {

    if (checkForCollision) {
      node->connected_Node_ptr.clear();
    }

    if (checkForCollision && collisionCheck(node->coord, base_radius)) {
      // remove node from NodeList
      invalidNodeList.push_back(node);
      continue;
    }

    nodes_pcl->points.push_back(
      pcl::PointXYZ(node->coord(0), node->coord(1), node->coord(2))
    );
  }
  
  if (checkForCollision && invalidNodeList.size() > 0) {
    for (NodePtr node: invalidNodeList) {
      auto finder = find(nodeList.begin(), nodeList.end(), node);
      if (finder != nodeList.end()) {
        nodeList.erase(finder);
      }
    }
  } 
}



void SkeletonFinder::run_postprocessing(double base_height, double base_radius, double connectionRadius, double tooCloseThreshold) {
  
  fillNodesPcl(NodeList);

  cout << "Number of nodes before removing too close nodes: " << NodeList.size() << endl;
  for (int nRemoved = removeTooCloseNodes(tooCloseThreshold); nRemoved > 0;  nRemoved = removeTooCloseNodes(tooCloseThreshold)) {
    cout << "Too close nodes removed: " << nRemoved << endl;
  }
  cout << "Number of nodes after removing too close nodes: " << NodeList.size() << endl;

  fillNodesPcl(NodeList, base_radius);

  cout << "Number of valid nodes: " << NodeList.size() << endl;

  // add edges where the path is clear
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
      
      auto start = Eigen::Vector3d(node->coord(0), node->coord(1), base_height);
      auto target = Eigen::Vector3d(nearest_node->coord(0), nearest_node->coord(1), base_height);
      if (checkPathClear(start, target)) {
        node->connected_Node_ptr.push_back(nearest_node);
        nearest_node->connected_Node_ptr.push_back(node);
        counter++;
      }
    }
  }

  cout << "Number of edges added: " << counter << endl;

  // calulate radius for each node->neighbour pair
  // --------------------------------------------
  //genSamplesOnUnitCircle();
  //for (NodePtr node: validNodeList) {
  //  for (NodePtr connected_node: node->connected_Node_ptr) {
  //    node->connected_Node_radius.push_back(
  //      calculateSafeRadius(node, connected_node));
  //  }
  
  
  // save valid nodes to file
  nodes_pcl->height = 1;
  nodes_pcl->width = nodes_pcl->points.size();
  pcl::io::savePCDFileASCII("/workspace/data_proc/data20/nodes_demo.pcd", *nodes_pcl);
  cout << "Nodes saved to file." << endl;

  std::fstream fs;
  fs.open("/workspace/data_proc/data20/connections_demo.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
  // save connections to file
  for (NodePtr cur_node : validNodeList) {
    for (NodePtr connect_node : cur_node->connected_Node_ptr) {
      auto finder = find(validNodeList.begin(), validNodeList.end(), connect_node);
      if (finder != validNodeList.end()) {
        fs << cur_node->coord(0) << " " << cur_node->coord(1) << " " << cur_node->coord(2) << "\n";
        fs << connect_node->coord(0) << " " << connect_node->coord(1) << " " << connect_node->coord(2) << "\n";
        fs << "\n";
      }
    }
  }

  cout << "Connections saved to file." << endl;


  Eigen::MatrixXd adjacencyMatrix = getAdjMatrix(validNodeList);
  cout << "Adjacency matrix size: " << adjacencyMatrix.rows() << "x" << adjacencyMatrix.cols() << endl;

  // save adjacency matrix to file (use file easy to open in python)
  ofstream adjacencyMatrixFile;
  adjacencyMatrixFile.open("/workspace/data_proc/data20/adjacency_matrix.txt");
  for (int i = 0; i < adjacencyMatrix.rows(); i++) {
    for (int j = 0; j < adjacencyMatrix.cols(); j++) {
      adjacencyMatrixFile << adjacencyMatrix(i, j) << " ";
    }
    adjacencyMatrixFile << endl;
  }



  // if we want to handle intersections as nodes
  // --------------------------------------------
  // handleInterSections(validNodeList, base_height);
  // removeTooCloseNodes(tooCloseCandidates, validNodeList);


  // spectral clustering
  // -----------------------
  //auto clusters = spectralClustering(validNodeList);
  //save_clusters(clusters, validNodeList);
  //cout << "Number of clusters: " << clusters.size() << endl;
  cout << "finalizing post-processing" << endl;

  // copy valid graph to NodeList
  NodeList.clear();
  cout << "nodelist cleared" << endl;
  nodes_pcl->clear();
  cout << "nodes_pcl cleared" << endl;

  for (NodePtr node : validNodeList) {
    NodeList.push_back(node);
    nodes_pcl->points.push_back(
      pcl::PointXYZ(node->coord(0), node->coord(1), node->coord(2))
    );
  }

  cout << "Post-processing finished." << endl;
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
      mergeNodes(node1, node2, NodeList, node1_w, node2_w);
    } else if (node1_canreplace_node2) {
      killOtherNode(node1, node2, NodeList);
    } else if (node2_canreplace_node1) {
      killOtherNode(node2, node1, NodeList);
    } else { //NOTE:: added this to always remove close nodes
      killOtherNode(node1, node2, NodeList);
    }
  }

  return tooCloseCandidates.size();
}



bool SkeletonFinder::canBeReplacedBy(NodePtr node_to_keep, NodePtr node_to_remove) {
  if (node_to_remove->connected_Node_ptr.size() == 1 && node_to_remove->connected_Node_ptr[0] == node_to_keep)
    return false;
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



// not sure about this
void SkeletonFinder::handleInterSections(vector<NodePtr> validNodeList, double base_height) {
  // (find intersecting edges, place a nearby node there if possible)
  // add all intersections and points and remove too close ones

  // checks connections between all the neighbours of all the nerighbours of the node
  // checks for intersections with node->neighbour connections
  vector<NodePtr> newNodes;
  for (NodePtr base_node: validNodeList) {
    for (NodePtr base_connected_node: base_node->connected_Node_ptr) {
      for (NodePtr base_connected_other_node: base_node->connected_Node_ptr) {
        if (base_connected_node == base_connected_other_node)
          continue;
        for (NodePtr other_ngb_node: base_connected_other_node->connected_Node_ptr) {
          // check if other_ngb_node is base_node or base_connected_node
          if (other_ngb_node == base_node || other_ngb_node == base_connected_node)
            continue;
          
          // check if base_node->base_connected_node intersects with base_connected_other_node->other_ngb_node
          Eigen::Vector2d base_to_base_connected_start(base_node->coord(0), base_node->coord(1));
          Eigen::Vector2d base_to_base_connected_end(base_connected_node->coord(0), base_connected_node->coord(1));
          Eigen::Vector2d other_connected_to_ngb_connected_start(base_connected_other_node->coord(0), base_connected_other_node->coord(1));
          Eigen::Vector2d other_connected_to_ngb_connected_end(other_ngb_node->coord(0), other_ngb_node->coord(1));
          Eigen::Vector2d intersection;

          bool has_intersect = calculateIntersection2D(
            base_to_base_connected_start, base_to_base_connected_end,
            other_connected_to_ngb_connected_start, other_connected_to_ngb_connected_end,
            intersection
          );

          if (has_intersect) {
            NodePtr newNode = new Node(Eigen::Vector3d(intersection.x(), intersection.y(), base_height), NULL, false);
            // should check for closeness to other nodes
            newNodes.push_back(newNode);

            // disconnect base_node and base_connected_node
            base_node->connected_Node_ptr.erase(
              remove(base_node->connected_Node_ptr.begin(), base_node->connected_Node_ptr.end(), base_connected_node),
              base_node->connected_Node_ptr.end()
            );
            base_connected_node->connected_Node_ptr.erase(
              remove(base_connected_node->connected_Node_ptr.begin(), base_connected_node->connected_Node_ptr.end(), base_node),
              base_connected_node->connected_Node_ptr.end()
            );
            // disconnect base_other_connected_node and other_ngb_node
            base_connected_other_node->connected_Node_ptr.erase(
              remove(base_connected_other_node->connected_Node_ptr.begin(), base_connected_other_node->connected_Node_ptr.end(), other_ngb_node),
              base_connected_other_node->connected_Node_ptr.end()
            );
            other_ngb_node->connected_Node_ptr.erase(
              remove(other_ngb_node->connected_Node_ptr.begin(), other_ngb_node->connected_Node_ptr.end(), base_connected_other_node),
              other_ngb_node->connected_Node_ptr.end()
            );

            // connect base_node and new node
            base_node->connected_Node_ptr.push_back(newNode);
            newNode->connected_Node_ptr.push_back(base_node);
            // connect new node and base_connected_node
            base_connected_node->connected_Node_ptr.push_back(newNode);
            newNode->connected_Node_ptr.push_back(base_connected_node);
            // base_connected_other node and new node
            base_connected_other_node->connected_Node_ptr.push_back(newNode);
            newNode->connected_Node_ptr.push_back(base_connected_other_node);
            // new node and other_ngb_node
            newNode->connected_Node_ptr.push_back(other_ngb_node);
            other_ngb_node->connected_Node_ptr.push_back(newNode);
          }
        }
      }
    }
  } 
}



bool SkeletonFinder::calculateIntersection2D(const Eigen::Vector2d& p1_start, const Eigen::Vector2d& p1_end,
                             const Eigen::Vector2d& p2_start, const Eigen::Vector2d& p2_end,
                             Eigen::Vector2d& intersection) {
    Eigen::Vector2d dir1 = p1_end - p1_start;
    Eigen::Vector2d dir2 = p2_end - p2_start;
    Eigen::Vector2d r = p1_start - p2_start;
    double denominator = dir1.x() * dir2.y() - dir1.y() * dir2.x();

    // If denominator is zero, the lines are parallel (or collinear)
    if (denominator == 0.0) {
        return false;  // No intersection, lines are parallel
    }

    double t = (r.x() * dir2.y() - r.y() * dir2.x()) / denominator;
    double u = (r.x() * dir1.y() - r.y() * dir1.x()) / denominator;

    // Check if t and u are within the valid range [0, 1] for both line segments
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        intersection = p1_start + t * dir1;
        return true;  // Intersection found
    }
    return false;  // No intersection within the segment bounds
}



Eigen::MatrixXd SkeletonFinder::getAdjMatrix(vector<NodePtr> validNodeList) {

  // could be optimized: only check for the upper triangle

  int num_nodes = validNodeList.size();
  cout << "Number of valid nodes: " << num_nodes << endl;
  Eigen::MatrixXd adjMatrix(num_nodes, num_nodes);
  // fill the adjacency matrix with 0s
  adjMatrix.setZero();

  for (int i = 0; i < num_nodes; i++) {
    for (int j = 0; j < num_nodes; j++) {
      NodePtr node1 = validNodeList[i];
      NodePtr node2 = validNodeList[j];

      if (i == j) {
        adjMatrix(i, j) = 1;
        continue;
      }

      auto finder = find(node1->connected_Node_ptr.begin(), node1->connected_Node_ptr.end(), node2);
      if (finder != node1->connected_Node_ptr.end()) {
        adjMatrix(i, j) = 1;
        adjMatrix(j, i) = 1;
      //} else {
      //  adjMatrix(i, j) = 0;
      //  adjMatrix(j, i) = 0;
      //}
      }
    }
  }
  return adjMatrix;
}

