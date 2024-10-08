#include <chrono>

#include "SkeletonFinder/skeleton_finder_3D.h"

using namespace Eigen;
using namespace std;

// run a-star path finding algorithm and return coordinates for path waypoints
pair<vector<Eigen::Vector3d>, vector<double>> SkeletonFinder::run_findpath(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z
) {
  Eigen::Vector3d start_query(_path_start_x, _path_start_y, _path_start_z);
  Eigen::Vector3d target_query(_path_target_x, _path_target_y, _path_target_z);
  auto begin = chrono::high_resolution_clock::now();
  vector<Eigen::Vector3d> path = findPath(start_query, target_query);
  auto finish = chrono::high_resolution_clock::now();
  if (path.empty()) {
    cout << "Find path failed!" << endl;
  } else {
    //visPath();
    cout << "Path finding time: " << chrono::duration_cast<chrono::duration<double>>(finish - begin).count() * 1000 << endl;
    double path_length = path_finder.pathLength(path);
    cout << "Path length: " << path_length << endl;
  }

  pair<vector<Eigen::Vector3d>, vector<double>> result(path, pathRadiuses(path));
  return result;
}

// run a-start as in run_findpath, but try to shorten the path by checking path greedily
// (optional: reverse path for backwards computation)
pair<vector<Eigen::Vector3d>, vector<double>> SkeletonFinder::run_findpath2(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z
) {
  auto path_w_radius = run_findpath(_path_start_x, _path_start_y, _path_start_z, _path_target_x, _path_target_y, _path_target_z);
  vector<Eigen::Vector3d> path = path_w_radius.first;
  vector<Eigen::Vector3d> new_path;

  Eigen::Vector3d this_waypoint(_path_start_x, _path_start_y, _path_start_z);
  Eigen::Vector3d next_waypoint;
  int achieved = -1;

  while (achieved < path.size()-1) {
    for (int i = achieved + 1; i < path.size(); i++) {
      next_waypoint = path[i];
      if (checkPathClear(this_waypoint, next_waypoint)) {
        achieved++;
        continue; 
      }
      break;
    }
    new_path.push_back(path[achieved]);
    this_waypoint = path[achieved];
  }
  pair<vector<Eigen::Vector3d>, vector<double>> result(new_path, pathRadiuses(new_path));
  return result;  
}

/*
// run a-start as in run_findpath, but try to shorten the path by checking path greedily
// (optional: reverse path for backwards computation)
pair<vector<Eigen::Vector3d>, vector<double>> SkeletonFinder::run_findpath3(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z,
  bool with_radius = true
) {
  vector<Eigen::Vector3d> path = run_findpath(_path_start_x, _path_start_y, _path_start_z, _path_target_x, _path_target_y, _path_target_z);
  vector<Eigen::Vector3d> new_path;

  
  Eigen::Vector3d this_waypoint(_path_start_x, _path_start_y, _path_start_z);
  int achieved = -1;

  while (achieved < path.size()-1) {
    for (int i = achieved + 1; i < path.size(); i++) {
      next_waypoint = path[i];
      if (checkPathClear(this_waypoint, next_waypoint)) {
        achieved++;
        continue;
      break;
    }
    new_path.push_back(path[achieved]);
    this_waypoint = path[achieved];
  }

  if (!with_radius)
    return new_path;
  return result;
}}
*/


vector<NodePtr> SkeletonFinder::pathToNodes(vector<Eigen::Vector3d> path) {
  vector<NodePtr> nodes;

  for (Eigen::Vector3d waypoint : path) {
    for (NodePtr node : NodeList) {
      if (isSamePos(node->coord, waypoint)) {
        nodes.push_back(node);
        break;
      }
    }
  }

  return nodes;
}


vector<double> SkeletonFinder::pathRadiuses(vector<Eigen::Vector3d> path) {
  vector<double> radiuses;
  vector<NodePtr> nodes = pathToNodes(path);

  for (size_t i = 0; i < nodes.size() - 1; i++) {
    NodePtr node = nodes[i];
    NodePtr next_node = nodes[i+1];
    auto finder = find(NodeList.begin(), NodeList.end(), next_node);

    // radius is cached in the node
    if (finder != NodeList.end()) {
      int index = distance(NodeList.begin(), finder);
      radiuses.push_back(node->connected_Node_radius[index]);
      continue;
    } 

    // calculate radius
    double radius = calculateSafeRadius(node, next_node);
    radiuses.push_back(radius);
  }

  return radiuses;
}



vector<Eigen::Vector3d> SkeletonFinder::findPath(Eigen::Vector3d start,
                                                 Eigen::Vector3d target) {
  Eigen::Vector3d start_astar = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_astar = Eigen::Vector3d::Zero();
  vector<Eigen::Vector3d> path;

  kdtreeForNodes.setInputCloud(nodes_pcl);

  pcl::PointXYZ pcl_start(start(0), start(1), start(2));
  pointIdxRadiusSearchForNodes.clear();
  pointRadiusSquaredDistanceForNodes.clear();
  kdtreeForNodes.nearestKSearch(pcl_start, 5, pointIdxRadiusSearchForNodes,
                                pointRadiusSquaredDistanceForNodes);
  for (std::size_t i = 0; i < pointIdxRadiusSearchForNodes.size(); ++i) {
    Eigen::Vector3d nbhd;
    nbhd << (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].x,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].y,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].z;
    if (checkSegClear(start, nbhd)) {
      start_astar = nbhd;
      break;
    }
  }

  pcl::PointXYZ pcl_target(target(0), target(1), target(2));
  pointIdxRadiusSearchForNodes.clear();
  pointRadiusSquaredDistanceForNodes.clear();
  kdtreeForNodes.nearestKSearch(pcl_target, 10, pointIdxRadiusSearchForNodes,
                                pointRadiusSquaredDistanceForNodes);
  for (std::size_t i = 0; i < pointIdxRadiusSearchForNodes.size(); ++i) {
    Eigen::Vector3d nbhd;
    nbhd << (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].x,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].y,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].z;
    // TODO
    //if (checkSegClear(target, nbhd)) {
    target_astar = nbhd;
    break;
    //}
  }

  if (start_astar == Eigen::Vector3d::Zero() ||
      target_astar == Eigen::Vector3d::Zero()) {
    cout << "Can't find nodes on skeleton to connect!" << endl;
    return path;
  }

  vector<Eigen::Vector3d> astar_path =
      findPathByAStar(start_astar, target_astar);


  if (!astar_path.empty()) {
    path.push_back(start);
    for (Eigen::Vector3d waypoint : astar_path)
      path.push_back(waypoint);
    path.push_back(target);
    cout << "Path found." << endl;
    //path_finder.visNodes();
    //path_finder.visConnections();
    return path;
  } else {
    cout << "Path not found!" << endl;
    return path;
  }
}

vector<Eigen::Vector3d> SkeletonFinder::findPathByAStar(Eigen::Vector3d start, Eigen::Vector3d target) {
  vector<a_star::NodePtr> as_nodes;
  for (NodePtr node : NodeList) {
    if (node->rollbacked)
      continue;

    vector<Eigen::Vector3d> connected_node_pos;
    for (NodePtr con_node : node->connected_Node_ptr) {
      // if (con_node->rollbacked) continue;
      connected_node_pos.push_back(con_node->coord);
    }
    a_star::NodePtr as_node = new a_star::Node(node->coord, connected_node_pos);
    as_nodes.push_back(as_node);
  }

  path_finder.init(as_nodes);
  int result = path_finder.search(start, target);
  vector<Eigen::Vector3d> path;
  if (result == 1) { // REACH_END
    // ROS_INFO("A_star: REACH_END!");
    path = path_finder.getPath();
  } else {
    // ROS_ERROR("A_star: NO_PATH!");
  }
  return path;
}


double SkeletonFinder::calculateSafeRadius(NodePtr node, NodePtr connected_node) {
  double radius = 0.0;
  bool safe = true;
  while (safe) {
    radius += _resolution;
    for (Eigen::Vector3d sample: sample_directions) {
      Eigen::Vector3d start = node->coord + sample * radius;
      Eigen::Vector3d target = connected_node->coord;
      if (!checkPathClear(start, target)) {
        safe = false;
        break;
      }
    } 
  }
  return radius - _resolution;
}

bool SkeletonFinder::checkPathClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  double length = (pos2 - pos1).norm();
  double step_length = _resolution;

  Eigen::Vector3d step = step_length * (pos2 - pos1) / length;
  int num_steps = ceil(length / step_length);

  Eigen::Vector3d begin_pos = pos1;
  for (int i = 0; i < num_steps; i++) {
    Eigen::Vector3d check_pos = begin_pos + i * step;
    if (!isValidPosition(check_pos))
      return false;
  }
  if (!isValidPosition(pos2))
    return false;
  return true;
}


bool SkeletonFinder::isValidPosition(Eigen::Vector3d base_pos) {
  Eigen::Vector3d downwards(0, 0, -1);
  double base_height = base_pos(2);
  pair<Vector3d, int> raycast_result = raycastOnRawMap(base_pos, downwards, 2*base_height);
  if (raycast_result.second == -2) {
    return false;
  }
  double floor_height = raycast_result.first(2); // 0
  if (floor_height > 2 * _search_margin) // should add floor z dim
    return false;
  return true;
}



