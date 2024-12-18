
#include "SkeletonFinder/skeleton_finder_3D.h"
#include <fstream>
#include <pcl/io/pcd_io.h>




void SkeletonFinder::saveNodes(const std::string& filename) {
    nodes_pcl->height = 1;
    nodes_pcl->width = nodes_pcl->points.size();
    pcl::io::savePCDFileASCII(filename, *nodes_pcl);
    cout << "Nodes saved to file: " << filename << endl;
}


void SkeletonFinder::saveConnections(const std::string& filename) {
    std::fstream fs;
    fs.open(filename, std::fstream::in | std::fstream::out | std::fstream::trunc);
    // save connections to file
    for (NodePtr cur_node : NodeList) {
        for (NodePtr connect_node : cur_node->connected_Node_ptr) {
        auto finder = find(NodeList.begin(), NodeList.end(), connect_node);
        if (finder != NodeList.end()) {
            fs << cur_node->coord(0) << " " << cur_node->coord(1) << " " << cur_node->coord(2) << "\n";
            fs << connect_node->coord(0) << " " << connect_node->coord(1) << " " << connect_node->coord(2) << "\n";
            fs << "\n";
        }
        }
    }
    cout << "Connections saved to file: " << filename << endl;
}


void SkeletonFinder::saveAdjMatrix(const std::string& filename) {
    Eigen::MatrixXd adjacencyMatrix = getAdjMatrix(NodeList);
    cout << "Adjacency matrix size: " << adjacencyMatrix.rows() << "x" << adjacencyMatrix.cols() << endl;

    // save adjacency matrix to file (use file easy to open in python)
    ofstream adjacencyMatrixFile;
    adjacencyMatrixFile.open(filename);
    for (int i = 0; i < adjacencyMatrix.rows(); i++) {
        for (int j = 0; j < adjacencyMatrix.cols(); j++) {
        adjacencyMatrixFile << adjacencyMatrix(i, j) << " ";
        }
        adjacencyMatrixFile << endl;
    }

    cout << "Adjacency matrix saved to file: " << filename << endl;
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


void SkeletonFinder::loadNodes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Convert the point cloud into Node objects
    NodeList.clear();
    for (const auto& point : cloud->points) {
        Eigen::Vector3d coord(point.x, point.y, point.z);
        NodePtr node = new Node(coord, NULL);
        recordNode(node);
    }
    cout << "Loaded " << nodes_pcl->points.size() << " nodes to pcl " << endl;
}


void SkeletonFinder::loadConnections(const std::string& filename) {
    std::ifstream adjacencyMatrixFile(filename);
    if (!adjacencyMatrixFile.is_open()) {
        throw std::runtime_error("Couldn't open file: " + filename);
    }

    std::vector<std::vector<double>> matrixData;
    std::string line;
    while (std::getline(adjacencyMatrixFile, line)) {
        std::istringstream iss(line);
        std::vector<double> row((std::istream_iterator<double>(iss)), std::istream_iterator<double>());
        matrixData.push_back(row);
    }
    adjacencyMatrixFile.close();

    // Convert vector of vectors to Eigen::MatrixXd
    size_t rows = matrixData.size();
    size_t cols = rows > 0 ? matrixData[0].size() : 0;
    Eigen::MatrixXd adjacencyMatrix(rows, cols);
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            adjacencyMatrix(i, j) = matrixData[i][j];
        }
    }

    cout << "Loaded adjacency matrix of size " << rows << "x" << cols << " from file: " << filename << endl;

    int num_nodes = NodeList.size();
    if (num_nodes != adjacencyMatrix.rows() || num_nodes != adjacencyMatrix.cols()) {
        throw std::runtime_error("Adjacency matrix size does not match number of nodes.");
    }

    for (int i = 0; i < num_nodes; ++i) {
        NodePtr node = NodeList[i];
        node->connected_Node_ptr.clear();
        for (int j = 0; j < num_nodes; ++j) {
            if (adjacencyMatrix(i, j) == 1 && i != j) { // Skip self-connections
                node->connected_Node_ptr.push_back(NodeList[j]);
            }
        }
    }
    cout << "Reconstructed connections between nodes." << endl;
}





/*
void SkeletonFinder::visConnections() {
    int node_count = 0;
    int connection_count = 0;

    int num_nodes = NodeList.size();
    for (int i = 0; i < num_nodes; i++) {
        NodePtr cur_node = NodeList.at(i);
        if (cur_node->rollbacked)
            continue;
        node_count++;

        if (cur_node->isGate)
            continue;
        
        int num_connect_nodes = cur_node->connected_Node_ptr.size();
        // if (cur_node->isGate) {
        //   ROS_WARN("(%f, %f, %f) is a GATE node having %d connections",
        //   cur_node->coord(0),
        //            cur_node->coord(1), cur_node->coord(2), num_connect_nodes);
        // } else {
        //   ROS_WARN("(%f, %f, %f) is a CENTER node having %d connections",
        //   cur_node->coord(0),
        //            cur_node->coord(1), cur_node->coord(2), num_connect_nodes);
        // }
        

        // filestream for saving connections
        std::fstream fs;
        fs.open("/workspace/ros2_ws/src/global_planner/resource/connections.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        for (int j = 0; j < num_connect_nodes; j++) {
            NodePtr connect_node = cur_node->connected_Node_ptr.at(j);

            // save edges to file
            
            fs << cur_node->coord(0) << " " << cur_node->coord(1) << " " << cur_node->coord(2) << "\n";
            fs << connect_node->coord(0) << " " << connect_node->coord(1) << " " << connect_node->coord(2) << "\n";
            fs << "\n";

            connection_count++;
        }
    }
}
*/
