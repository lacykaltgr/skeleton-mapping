
#include "SkeletonFinder/skeleton_finder_3D.h"
#include <fstream>
#include <pcl/io/pcd_io.h>


// Add this new function to handle file writing
void SkeletonFinder::saveToFile(const std::string& filename, const std::string& data) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << data;
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}

void SkeletonFinder::visualization() {
    //visStart();
    visNodesAndVertices();
    //visPolygons();
    //visFrontiers();
    //visMap();
    visConnections();
}

void SkeletonFinder::visNodesAndVertices() {
    // nodes_pcl.clear();
    //black_vertices_pcl.clear();
    //white_vertices_pcl.clear();
    //grey_vertices_pcl.clear();
    
    /*
    int num_nodes = NodeList.size();
    for (int i = num_nodes - 1; i >= 0; i--) {
        if (NodeList.at(i)->rollbacked)
            continue;
        if (NodeList.at(i)->isGate)
            continue;
        // nodes_pcl.points.push_back(pcl::PointXYZ(NodeList.at(i)->coord(0),
        // NodeList.at(i)->coord(1),
        //                                          NodeList.at(i)->coord(2)));

        for (VertexPtr v : NodeList.at(i)->black_vertices) {
            if (v->type == BLACK) {
                black_vertices_pcl.points.push_back(
                    pcl::PointXYZ(v->coord(0), v->coord(1), v->coord(2)));
            } else {
                grey_vertices_pcl.points.push_back(
                    pcl::PointXYZ(v->coord(0), v->coord(1), v->coord(2)));
            }
        }
        for (VertexPtr v : NodeList.at(i)->white_vertices) {
            white_vertices_pcl.points.push_back(
                pcl::PointXYZ(v->coord(0), v->coord(1), v->coord(2)));
        }

        if (!_visualize_all)
            break;
    }
    */

    // Save point clouds to PCD files
    size_t num_points = nodes_pcl->size();
    nodes_pcl->height = 1;
    nodes_pcl->width = num_points;
    cout << "num points: " << num_points << endl;

    /*
    std::cout << "Points:" << std::endl;
    for (const auto& point : nodes_pcl.points) {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }

    std::cout << "Black vertices:" << std::endl;
    for (const auto& point : black_vertices_pcl.points) {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }

    std::cout << "White vertices:" << std::endl;
    for (const auto& point : white_vertices_pcl.points) {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }

    std::cout << "Grey vertices:" << std::endl;
    for (const auto& point : grey_vertices_pcl.points) {
        std::cout << point.x << " " << point.y << " " << point.z << std::endl;
    }
    */

    pcl::io::savePCDFileASCII("/workspace/ros2_ws/src/global_planner/resource/nodes.pcd", (*nodes_pcl));
    
    //pcl::io::savePCDFileASCII("black_vertices.pcd", black_vertices_pcl);
    //pcl::io::savePCDFileASCII("white_vertices.pcd", white_vertices_pcl);
    //pcl::io::savePCDFileASCII("grey_vertices.pcd", grey_vertices_pcl);
    
}

void SkeletonFinder::visMap() {
    //pcl::io::savePCDFileASCII("map.pcd", vis_map_pcl);
}

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
