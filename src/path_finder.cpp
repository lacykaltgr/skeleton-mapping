#include <chrono>

#include "SkeletonFinder/skeleton_finder_3D.h"

using namespace Eigen;
using namespace std;

// run a-star path finding algorithm and return coordinates for path waypoints
pair<vector<Eigen::Vector3d>, pair<double, double>> SkeletonFinder::run_findpath_w_stats(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z
) {
  Eigen::Vector3d start_query(_path_start_x, _path_start_y, _path_start_z);
  Eigen::Vector3d target_query(_path_target_x, _path_target_y, _path_target_z);

  auto begin = chrono::high_resolution_clock::now();
  vector<Eigen::Vector3d> path = findPath(start_query, target_query);
  auto finish = chrono::high_resolution_clock::now();

  double path_find_time;
  double path_length;

  if (path.empty()) {
    cout << "Find path failed!" << endl;
    path_find_time = -1;
    path_length = -1;
  } else {
    path_find_time = chrono::duration_cast<chrono::duration<double>>(finish - begin).count() * 1000;
    path_length = path_finder.pathLength(path);

    cout << "Path finding time: " << chrono::duration_cast<chrono::duration<double>>(finish - begin).count() * 1000 << endl;
    cout << "Path length: " << path_length << endl;
  }
  pair<double, double> stats(path_find_time, path_length);
  pair<vector<Eigen::Vector3d>, pair<double, double>> result(path, stats);
  return result; 
}


pair<vector<Eigen::Vector3d>, vector<double>> SkeletonFinder::run_findpath(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z
) {
  auto path_w_stats = run_findpath_w_stats(_path_start_x, _path_start_y, _path_start_z, _path_target_x, _path_target_y, _path_target_z);
  auto path = path_w_stats.first;

  vector<double> path_radiuses;
  for (size_t i = 0; i < path.size() - 1; i++) {
    path_radiuses.push_back(0.5);
  }
  pair<vector<Eigen::Vector3d>, vector<double>> result(path, path_radiuses);

  //pair<vector<Eigen::Vector3d>, vector<double>> result(path, pathRadiuses(path));
  return result;
}

// run a-start as in run_findpath, but try to shorten the path by checking path greedily
pair<vector<Eigen::Vector3d>, vector<double>> SkeletonFinder::run_findpath_shorten(
  double _path_start_x, double _path_start_y, double _path_start_z,
  double _path_target_x, double _path_target_y, double _path_target_z
) {
  auto path_w_stats = run_findpath_w_stats(_path_start_x, _path_start_y, _path_start_z, _path_target_x, _path_target_y, _path_target_z);
  vector<Eigen::Vector3d> path = path_w_stats.first;
  Eigen::Vector3d start(_path_start_x, _path_start_y, _path_start_z);
  vector<Eigen::Vector3d> new_path = shortenPath(start, path);

  pair<vector<Eigen::Vector3d>, vector<double>> result(new_path, pathRadiuses(new_path));
  return result;  
}



vector<Eigen::Vector3d> SkeletonFinder::shortenPath(Eigen::Vector3d start, vector<Eigen::Vector3d> path) {

  vector<Eigen::Vector3d> new_path;

  Eigen::Vector3d this_waypoint = start;
  Eigen::Vector3d next_waypoint;
  // cast path.size() to int
  int size = path.size();
  int achieved = -1;

  while (achieved < size-1) {
    achieved++;
    for (int i = achieved + 1; i < size; i++) {
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

  return new_path;
}


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
  genSamplesOnUnitCircle();

  for (size_t i = 0; i < nodes.size() - 1; i++) {
    NodePtr node = nodes[i];
    NodePtr next_node = nodes[i+1];
    //auto finder = find(NodeList.begin(), NodeList.end(), next_node);

    // radius is cached in the node
    //if (finder != NodeList.end()) {
    //  int index = distance(NodeList.begin(), finder);
    //  radiuses.push_back(node->connected_Node_radius[index]);
    //  continue;
    //} 

    // calculate radius
    double radius = 0.5; // calculateSafeRadius(node, next_node);
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
    //if (checkSegClear(start, nbhd)) {
    start_astar = nbhd;
    break;
    //}
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


vector<Eigen::Vector3d> SkeletonFinder::findPathBetweenObjects(Eigen::Vector3d start,
                                                               Eigen::Vector3d target,
                                                                Eigen::Vector3d start_bbox_extent,
                                                                Eigen::Vector3d start_box_center,
                                                                Eigen::Vector3d target_bbox_extent,
                                                                Eigen::Vector3d target_box_center) {
  Eigen::Vector3d start_astar = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_astar = Eigen::Vector3d::Zero();
  vector<Eigen::Vector3d> path;

  kdtreeForNodes.setInputCloud(nodes_pcl);


  pcl::PointXYZ pcl_start(start(0), start(1), start(2));
  pointIdxRadiusSearchForNodes.clear();
  pointRadiusSquaredDistanceForNodes.clear();
  kdtreeForNodes.nearestKSearch(pcl_start, 10, pointIdxRadiusSearchForNodes,
                                pointRadiusSquaredDistanceForNodes);
  for (std::size_t i = 0; i < pointIdxRadiusSearchForNodes.size(); ++i) {
    Eigen::Vector3d nbhd;
    nbhd << (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].x,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].y,
        (*nodes_pcl)[pointIdxRadiusSearchForNodes[i]].z;
    double path_clear_len = rayFlyingLen(nbhd, start, _base_radius) + _resolution;
    Eigen::Vector3d path_clear_dir = (start - nbhd).normalized();
    Eigen::Vector3d hit_point = nbhd + path_clear_len * path_clear_dir;

    // check if hit point is inside the bounding box
    if (hit_point(0) >= start_box_center(0) - start_bbox_extent(0) - _resolution &&
        hit_point(0) <= start_box_center(0) + start_bbox_extent(0) + _resolution &&
        hit_point(1) >= start_box_center(1) - start_bbox_extent(1) - _resolution &&
        hit_point(1) <= start_box_center(1) + start_bbox_extent(1) + _resolution &&
        hit_point(2) >= start_box_center(2) - start_bbox_extent(2) - _resolution &&
        hit_point(2) <= start_box_center(2) + start_bbox_extent(2) + _resolution) {
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
    
    double path_clear_len = rayFlyingLen(nbhd, target, _base_radius) + _resolution;
    Eigen::Vector3d path_clear_dir = (target - nbhd).normalized();
    Eigen::Vector3d hit_point = nbhd + path_clear_len * path_clear_dir;

    // check if hit point is inside the bounding box
    if (hit_point(0) >= target_box_center(0) - target_bbox_extent(0) - _resolution &&
        hit_point(0) <= target_box_center(0) + target_bbox_extent(0) + _resolution &&
        hit_point(1) >= target_box_center(1) - target_bbox_extent(1) - _resolution &&
        hit_point(1) <= target_box_center(1) + target_bbox_extent(1) + _resolution &&
        hit_point(2) >= target_box_center(2) - target_bbox_extent(2) - _resolution &&
        hit_point(2) <= target_box_center(2) + target_bbox_extent(2) + _resolution) {
      target_astar = nbhd;
      break;
    }
  }

  if (start_astar == Eigen::Vector3d::Zero() ||
      target_astar == Eigen::Vector3d::Zero()) {
    return path;
  }

  vector<Eigen::Vector3d> astar_path =
      findPathByAStar(start_astar, target_astar);

  if (!astar_path.empty()) {
    path.push_back(start);
    for (Eigen::Vector3d waypoint : astar_path)
      path.push_back(waypoint);
    path.push_back(target);
  } else {
    cout << "Path not found!" << endl;
  }
  return path;
}



vector<Eigen::Vector3d> SkeletonFinder::findPathByAStar(Eigen::Vector3d start, Eigen::Vector3d target) {
  vector<a_star::NodePtr> as_nodes;
  for (NodePtr node : NodeList) {
    if (node->rollbacked)
       continue;

    vector<Eigen::Vector3d> connected_node_pos;
    for (NodePtr con_node : node->connected_Node_ptr) {
      // TODO: this line needed?
      if (con_node->rollbacked) continue;
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
  if (radius > 2 * _base_radius) {
    radius = 2 * _base_radius;
  }
  return radius - _resolution;
}

bool SkeletonFinder::checkPathClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  // check if path is clear
  // rayflying for aerial robots, raywalking for ground robots
  if (_robot_type == 0) {
    return canRayWalk(pos1, pos2, _base_radius, _raywalking_max_height_diff);
  } else if (_robot_type == 1) {
    return canRayFly(pos1, pos2, _base_radius);
  } else {
    cout << "Robot type not supported!" << endl;
    return false;
  }
}


double SkeletonFinder::checkPathClearLength(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  // check the length of the clear path from pos1 to pos2
  // rayflying for aerial robots, raywalking for ground robots
  if (_robot_type == 0) {
    return rayWalkingLen(pos1, pos2, _base_radius, _raywalking_max_height_diff);
  } else if (_robot_type == 1) {
    return rayFlyingLen(pos1, pos2, _base_radius);
  } else {
    cout << "Robot type not supported!" << endl;
    return false;
  }
}


bool SkeletonFinder::checkPathClearAndValid(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  if (_robot_type == 0) {
    return canRayWalk(pos1, pos2, _base_radius, _raywalking_max_height_diff);
  } else if (_robot_type == 1) {
    bool canRayFly = canRayFly(pos1, pos2, _base_radius);
    bool canRayFlyValid = canRayFlyValid(pos1, pos2, _base_radius, _min_hit_ratio);
    return canRayFly && canRayFlyValid;
  } else {
    cout << "Robot type not supported!" << endl;
    return false;
  }
}

// return <clear length, valid length pair>
pair<double, double> SkeletonFinder::checkPathClearAndValidLength(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  if (_robot_type == 0) {
    // maybe could check with larger height diff to differentiate between valid and clear path
    double rayWalkLen = rayWalkingLen(pos1, pos2, _base_radius, _raywalking_max_height_diff);
    return pair<double, double>(rayWalkLen, rayWalkLen);
  } else if (_robot_type == 1) {
    double valid = rayFlyingValidLen(pos1, pos2, _base_radius, _min_hit_ratio);
    double clear = rayFlyingLen(pos1, pos2, _base_radius);
    if (valid > clear) valid = clear;
    return pair<double, double>(clear, valid);
  } else {
    cout << "Robot type not supported!" << endl;
    return std::numeric_limits<double>::min();
  }
}



bool SkeletonFinder::canRayFly(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius) {
  return fabs(rayFlyingLen(pos1, pos2, base_radius) - (pos2 - pos1).norm()) < 1e-4;
}

bool SkeletonFinder::canRayFlyValid(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double min_hit_ratio) {
  return fabs(rayFlyingValidLen(pos1, pos2, base_radius, min_hit_ratio) - (pos2 - pos1).norm()) < 1e-4;
}

bool SkeletonFinder::canRayWalk(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double max_step_height=0.2) {
  return fabs(rayWalkingLen(pos1, pos2, base_radius, max_step_height) - (pos2 - pos1).norm()) < 1e-4;
}



double SkeletonFinder::rayFlyingLen(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius) {
  double length = (pos2 - pos1).norm();
  double step_length = _resolution;

  Eigen::Vector3d step = step_length * (pos2 - pos1) / length;
  int num_steps = ceil(length / step_length);

  Eigen::Vector3d begin_pos = pos1;
  for (int i = 0; i < num_steps; i++) {
    Eigen::Vector3d check_pos = begin_pos + i * step;
    if (collisionCheck(check_pos, base_radius)) {
      return (check_pos - begin_pos).norm() - step_length;
    }
  }
  if (collisionCheck(pos2, base_radius)) {
    return (pos2 - begin_pos).norm() - step_length;
  }
  return (pos2 - begin_pos).norm();
}

double SkeletonFinder::rayFlyingValidLen(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double min_hit_ratio) {
  double length = (pos2 - pos1).norm();
  double step_length = _resolution;

  Eigen::Vector3d step = step_length * (pos2 - pos1) / length;
  int num_steps = ceil(length / step_length);

  Eigen::Vector3d forward = (pos2 - pos1).normalized();
  Eigen::Vector3d begin_pos = pos1;
  for (int i = 0; i < num_steps; i++) {
    Eigen::Vector3d check_pos = begin_pos + i * step;
    //if (collisionCheck(check_pos, base_radius)) {
    //  return (check_pos - begin_pos).norm() - step_length;
    //}

    // check if hit ratio is enough -> path is valid
    if (i > 0) {
      double hit_ratio = checkHitRatio(check_pos, forward, base_radius);
      if (hit_ratio < min_hit_ratio) {
        return (check_pos - begin_pos).norm() - step_length;
      }
    }
  }
  //if (collisionCheck(pos2, base_radius)) {
  //  return (pos2 - begin_pos).norm() - step_length;
  //}
  return (pos2 - begin_pos).norm();
}


double SkeletonFinder::rayWalkingLen(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double max_step_height=0.2) {
  // check first position
  int max_distance_to_floor = 3.0;
  double begin_floor_height = checkFloorHeight(pos1, base_radius, max_distance_to_floor);
  if (begin_floor_height == std::numeric_limits<double>::min()) return 0;
  double target_floor_height = checkFloorHeight(pos2, base_radius, max_distance_to_floor);

  Eigen::Vector3d begin_pos(pos1(0), pos1(1), begin_floor_height + _base_height);
  Eigen::Vector3d target_pos(pos2(0), pos2(1), target_floor_height + _base_height);

  double length = (target_pos - begin_pos).norm();
  double length_pos = (pos2 - pos1).norm();
  double step_length = _resolution;

  Eigen::Vector3d step = step_length * (target_pos - begin_pos) / length;
  Eigen::Vector3d step_pos = step_length * (pos2 - pos1) / length_pos;
  int num_steps = ceil(length / step_length);

  // check all positions in path  
  double ref_floor_height = begin_floor_height;
  for (int i = 1; i < num_steps; i++) {
    Eigen::Vector3d check_pos = begin_pos + i * step;
    double check_floor_height = checkFloorHeight(check_pos, base_radius, max_distance_to_floor);

    if (check_floor_height == std::numeric_limits<double>::min() || 
          fabs(check_floor_height - ref_floor_height) > max_step_height) {
      Eigen::Vector3d real_pos = pos1 + i * step_pos;
      return (i * step_pos).norm() - step_length;
    }

    ref_floor_height = check_floor_height;
    
  }

  // check the last position
  if (target_floor_height == std::numeric_limits<double>::min()) {
    return length_pos - step_length;
  }
  if (fabs(target_floor_height - ref_floor_height) > max_step_height) {
    return length_pos - step_length;
  }

  return length_pos;
}


double SkeletonFinder::checkFloorHeight(Eigen::Vector3d base_pos, double base_radius, double cut_off_length = 3.0) {
  Eigen::Vector3d downwards(0, 0, -1);
  pair<Vector3d, int> raycast_result = raycastOnRawMap(base_pos, downwards, cut_off_length, base_radius);
  if (raycast_result.second == -2) {
    // minimum value
    return std::numeric_limits<double>::min();
  }
  double floor_height = raycast_result.first(2);
  return floor_height;
}

double SkeletonFinder::checkHitRatio(Eigen::Vector3d base_pos, Eigen::Vector3d forward_dir, double base_radius, double cut_off_length = 3.0) {
  Eigen::Vector3d downwards(0, 0, -1);
  double phi = 2 * M_PI; 
  double x, y, z, theta;
  // circle perimeter / resolution = sampling density
  int num_samples = (int) phi * cut_off_length / _search_margin;
  int num_hits = 0;
  

  for (int i = 0; i < num_samples; i++) {
    theta = phi * ((double)i / num_samples);  // Angle for each sample

    // Compute z (up) and y (right) on the unit circle
    x = 0.0;
    y = cos(theta);
    z = sin(theta);

    Eigen::Vector3d sample_direction(x, y, z);
    Eigen::Vector3d direction = forward_dir + sample_direction;
    pair<Vector3d, int> raycast_result = raycastOnRawMap(base_pos, direction, cut_off_length, _search_margin);
    if (raycast_result.second != -2) {
      num_hits++;
    }
  }

  return (double)num_hits / num_samples;
}



