#include "SkeletonFinder/skeleton_finder_3D.h"
#include <algorithm>
#include <chrono>
#include <fstream>

using namespace Eigen;
using namespace std;

namespace backward {
backward::SignalHandling sh;
}

SkeletonFinder::SkeletonFinder(YAML::Node &config) {

  setParam(
    config["x_min"].as<double>(), config["x_max"].as<double>(),
    config["y_min"].as<double>(), config["y_max"].as<double>(),
    config["z_min"].as<double>(), config["z_max"].as<double>(),
    config["start"]["x"].as<double>(), config["start"]["y"].as<double>(), config["start"]["z"].as<double>(),
    config["frontier_creation_threshold"].as<double>(), config["frontier_jump_threshold"].as<double>(), config["frontier_split_threshold"].as<double>(),
    config["min_flowback_creation_threshold"].as<int>(), config["min_flowback_creation_radius_threshold"].as<double>(),
    config["min_node_radius"].as<double>(), config["search_margin"].as<double>(), config["max_ray_length"].as<double>(),
    config["max_expansion_ray_length"].as<double>(), config["max_height_diff"].as<double>(), config["sampling_density"].as<int>(),
    config["max_facets_grouped"].as<int>(), config["resolution"].as<double>(), config["bad_loop"].as<bool>(),
    config["base_height"].as<double>(), config["base_radius"].as<double>(), config["connection_radius"].as<double>(), config["too_close_threshold"].as<double>(),
    config["robot_type"].as<int>(), config["raywalking_max_height_diff"].as<double>(), config["min_hit_ratio"].as<double>(), config["exploration_mode"].as<bool>()
  );

  nodes_pcl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

SkeletonFinder::~SkeletonFinder() {}


bool SkeletonFinder::isSamePos(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  return getDis(pos1, pos2) < 1e-4;
}

void SkeletonFinder::reset() {
  for (int i = 0; i < int(NodeList.size()); i++) {
    NodePtr ptr = NodeList[i];
    delete ptr;
  }
  NodeList.clear();
}

void SkeletonFinder::setStartPt(Vector3d startPt) {
  NodePtr start = new Node(startPt, NULL);
  initNode(start);
}


inline pair<double, int> SkeletonFinder::radiusSearch(Vector3d &search_Pt) {
  double min_dis = radiusSearchOnRawMap(search_Pt);
  int min_dis_node_index = -1;

  if (min_dis < _search_margin) {
    pair<double, int> return_pair(min_dis, min_dis_node_index);
    return return_pair;
  }

  pcl::PointXYZ searchPoint;
  searchPoint.x = search_Pt(0);
  searchPoint.y = search_Pt(1);
  searchPoint.z = search_Pt(2);

  for (NodePtr node : NodeList) {
    if (node->rollbacked || node->isGate)
      continue;

    if (getDis(node->original_coord, search_Pt) > _max_ray_length + min_dis)
      continue;

    pointIdxRadiusSearchForRawMap.clear();
    pointRadiusSquaredDistanceForRawMap.clear();

    kdtreesForPolys.at(node->index)
        ->nearestKSearch(searchPoint, 1, pointIdxRadiusSearchForRawMap,
                         pointRadiusSquaredDistanceForRawMap);
    double radius = sqrt(pointRadiusSquaredDistanceForRawMap[0]);

    if (radius < min_dis) {
      min_dis = radius;
      min_dis_node_index = node->index;
    }
  }

  pair<double, int> return_pair(min_dis, min_dis_node_index);
  return return_pair;
}

double SkeletonFinder::radiusSearchOnRawMap(Vector3d &search_Pt) {

  pcl::PointXYZ searchPoint;
  searchPoint.x = search_Pt(0);
  searchPoint.y = search_Pt(1);
  searchPoint.z = search_Pt(2);

  pointIdxRadiusSearchForRawMap.clear();
  pointRadiusSquaredDistanceForRawMap.clear();

  kdtreeForRawMap.nearestKSearch(searchPoint, 1, pointIdxRadiusSearchForRawMap,
                                 pointRadiusSquaredDistanceForRawMap);
  double radius = sqrt(pointRadiusSquaredDistanceForRawMap[0]);
  //   return min(radius, double(max_radius));
  return radius;
}

bool SkeletonFinder::collisionCheck(Vector3d search_pt, double threshold) {
  // Point cloud map
  double dis = radiusSearchOnRawMap(search_pt);
  if (dis < threshold)
    return true;
  else
    return false;
}


void SkeletonFinder::recordNode(NodePtr new_node) {
  NodeList.push_back(new_node);
  nodes_pcl->points.push_back(pcl::PointXYZ(
      new_node->coord(0), new_node->coord(1), new_node->coord(2)));

  if (!new_node->isGate) {
    int index = center_NodeList.size();
    new_node->index = index;
    center_NodeList.push_back(new_node);
  }
}

/* ------------------------------------------------------ */


void SkeletonFinder::run_processing(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl) {

  if (raw_map_pcl->points.size() == 0) {
    cout << "Map pointcloud is empty!" << endl;
    return;
  }
  findBoundingBox(raw_map_pcl);
  //TODO: addBbxToMap
  //addBbxToMap(raw_map_pcl);


  // Point cloud map
  kdtreeForRawMap.setInputCloud(raw_map_pcl);
    
  Eigen::Vector3d start;
  start << _start_x, _start_y, _start_z;

  cout << "Generating skeleton..." << endl;
  auto begin = chrono::high_resolution_clock::now();
  skeletonExpansion(start);
  auto finish = chrono::high_resolution_clock::now();

  cout << "Expansion finished." << endl;
  double timing = chrono::duration_cast<chrono::duration<double>>(finish - begin).count();

  cout << "Expansion time: " << timing << endl;

  cout << "Number of nodes: " << NodeList.size() << endl;
}





void SkeletonFinder::skeletonExpansion(Eigen::Vector3d startPt) {
  genSamplesOnUnitSphere();
  identifyBwFacets();
  setStartPt(startPt);

  FrontierPtr cur_frontier;
  while (!pending_frontiers.empty()) {
    auto begin = chrono::high_resolution_clock::now();

    cur_frontier = pendingFrontiersPopFront();
    if (cur_frontier == NULL)
      continue;

    verifyFrontier(cur_frontier);

    if (!cur_frontier->valid) {
      Eigen::Vector3d prev_proj_center = cur_frontier->proj_center;
      Eigen::Vector3d prev_normal = cur_frontier->outwards_unit_normal;
      cur_frontier->proj_center = cur_frontier->proj_facet->center;
      verifyFrontier(cur_frontier);

      if (!cur_frontier->valid) {
        cur_frontier->proj_center = prev_proj_center;
        cur_frontier->outwards_unit_normal =
            cur_frontier->proj_facet->outwards_unit_normal;
        verifyFrontier(cur_frontier);

        if (!cur_frontier->valid) {
          cur_frontier->proj_center = cur_frontier->proj_facet->center;
          verifyFrontier(cur_frontier);

          if (!cur_frontier->valid) {
            cur_frontier->outwards_unit_normal = prev_normal;
            for (FacetPtr candidate_facet :
                 cur_frontier->proj_facet->nbhd_facets) {
              cur_frontier->proj_center = candidate_facet->center;
              verifyFrontier(cur_frontier);

              if (!cur_frontier->valid) {
                cur_frontier->outwards_unit_normal =
                    candidate_facet->outwards_unit_normal;
                verifyFrontier(cur_frontier);
                if (cur_frontier->valid)
                  break;
              } else {
                break;
              }
            }
          }
        }
      }
    }
    auto finish = chrono::high_resolution_clock::now();
    verifyFrontier_timing += chrono::duration_cast<chrono::duration<double>>(finish - begin).count();
    begin = chrono::high_resolution_clock::now();
    if (cur_frontier->valid) {
      processFrontier(cur_frontier);
    }
    finish = chrono::high_resolution_clock::now();
    processFrontier_timing += chrono::duration_cast<chrono::duration<double>>(finish - begin).count();
  }
}

bool SkeletonFinder::initNode_0(NodePtr curNodePtr) {
  auto begin = chrono::high_resolution_clock::now();
  auto prev = begin;
  chrono::high_resolution_clock::time_point curr;

  if (!checkWithinBbx(curNodePtr->coord)) {
    return false;
  }

  if (!checkFloor(curNodePtr)) {
    return false;
  }

  if (!curNodePtr->isGate) {

    genBlackAndWhiteVertices(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    genBlackAndWhiteVertices_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();
    prev = curr;

    if (curNodePtr->black_vertices.size() < 4) {
      return false;
    }

    centralizeNodePos(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    centralizeNodePos_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();
    prev = curr;

    double radius = getNodeRadius(curNodePtr);
    if (radius < _min_node_radius && curNodePtr->white_vertices.empty()) {
      return false;
    }

    // Absorb node inside node
    if (curNodePtr->seed_frontier != NULL) {
      bool diff_ind = false;
      int ind = curNodePtr->black_vertices.at(0)->collision_node_index;
      for (VertexPtr v : curNodePtr->black_vertices) {
        if (v->collision_node_index != ind) {
          diff_ind = true;
          break;
        }
      }
      if (!diff_ind)
        return false;
    }

    findFlowBack(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    findFlowBack_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();
    prev = curr;

    identifyFacets(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    identifyFacets_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();
    prev = curr;

    identifyFrontiers(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    identifyFrontiers_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();
    prev = curr;

    addFacetsToPcl(curNodePtr);

    curr = chrono::high_resolution_clock::now();
    addFacetsToPcl_timing += chrono::duration_cast<chrono::duration<double>>(curr - prev).count();

  }

  // curNodePtr->clearance = radiusSearchOnRawMap(curNodePtr->coord);
  // visSphere(curNodePtr->coord, curNodePtr->clearance);
  recordNode(curNodePtr);

  return true;
}

void SkeletonFinder::genBlackAndWhiteVertices(NodePtr nodePtr) {
  // vector<VertexPtr> black;
  vector<Eigen::Vector3d>::iterator it;
  for (it = sample_directions.begin(); it != sample_directions.end(); it++) {
    int index = nodePtr->sampling_directions.size();
    nodePtr->sampling_directions.push_back(vec3((*it)(0), (*it)(1), (*it)(2)));

    pair<Vector3d, int> raycast_result =
        raycast(nodePtr->coord, *it, _max_ray_length);
    Eigen::Vector3d newVertex = raycast_result.first;
    if (raycast_result.second == -2) {
      newVertex += (*it) * _max_ray_length;
      VertexPtr new_white_vertex = new Vertex(newVertex, (*it), WHITE);
      new_white_vertex->sampling_dire_index = index;
      nodePtr->white_vertices.push_back(new_white_vertex);
    } else {
      VertexPtr new_black_vertex;
      new_black_vertex = new Vertex(newVertex, (*it), BLACK);
      new_black_vertex->collision_node_index = raycast_result.second;
      new_black_vertex->sampling_dire_index = index;
      new_black_vertex->dis_to_center =
          getDis(new_black_vertex->coord, nodePtr->coord);
      nodePtr->black_vertices.push_back(new_black_vertex);
    }
  }
}


void SkeletonFinder::genSamplesOnUnitSphere() {
  sample_directions.clear();
  // Fibonicci sphere
  double phi = M_PI * (3 - sqrt(5));
  double x, y, z, radius, theta;

  for (int i = 0; i < _sampling_density; i++) {
    y = 1 - 2 * ((float)i / (float)(_sampling_density - 1));
    radius = sqrt(1 - y * y);
    theta = phi * i;
    x = cos(theta) * radius;
    z = sin(theta) * radius;

    Eigen::Vector3d sample;
    sample << x, y, z;
    sample_directions.push_back(sample);
  }
}

void SkeletonFinder::genSamplesOnUnitCircle() {
  sample_directions.clear();
  double phi = 2 * M_PI; 
  double x, y, theta;

  for (int i = 0; i < _sampling_density; i++) {
    theta = phi * ((double)i / _sampling_density);  // Angle for each sample

    // Compute x and y on the unit circle
    x = cos(theta);
    y = sin(theta);

    Eigen::Vector3d sample;
    sample << x, y, 0.0;
    sample_directions.push_back(sample);  // Store the 2D sample
  }
}



// -2: collision not found within cut_off_length
// -1: collision is with map
// >=0: collision is with the node of that index
pair<Vector3d, int> SkeletonFinder::raycast(Vector3d ray_source,
                                            Vector3d direction,
                                            double cut_off_length) {
  double clearance = radiusSearch(ray_source).first;
  if (clearance < cut_off_length) {
    Eigen::Vector3d current_pos = ray_source + clearance * direction;
    double length = clearance;
    while (length <= cut_off_length) {
      pair<double, int> rs = radiusSearch(current_pos);
      double radius = rs.first;
      if (radius < _search_margin) {
        pair<Vector3d, int> return_pair(current_pos, rs.second);
        return return_pair;
      }
      current_pos += radius * direction;
      length += radius;
    }
  }

  // point is achieveable by raycasting
  // try raywalking
  // TODO: added this part
  
  if (_exploration_mode) {
    Vector3d target_pos = ray_source + cut_off_length * direction;
    double clearence = checkPathClearLength(ray_source, target_pos);
    bool safe = fabs(clearence - (target_pos - ray_source).norm()) < 1e-4;
    if (!safe) {
      double safe_len = clearence - _base_radius;
      if (safe_len < 0) safe_len = 0;
      pair<Vector3d, int> return_pair(ray_source + direction * (safe_len), -1);
      return return_pair;
    }
  }

  
  pair<Vector3d, int> return_pair(ray_source, -2);
  return return_pair;
}

// -2: collision not found within cut_off_length
// -1: collision is with map
pair<Vector3d, int> SkeletonFinder::raycastOnRawMap(Vector3d ray_source,
                                                    Vector3d direction,
                                                    double cut_off_length) {
  // Point cloud map
  double clearance = radiusSearchOnRawMap(ray_source);
  if (clearance > cut_off_length) {
    pair<Vector3d, int> return_pair(ray_source, -2);
    return return_pair;
  } else {
    Eigen::Vector3d current_pos = ray_source + clearance * direction;
    double length = clearance;

    while (length <= cut_off_length) {
      double radius = radiusSearchOnRawMap(current_pos);

      if (radius < _search_margin) {
        pair<Vector3d, int> return_pair(current_pos, -1);
        return return_pair;
      }
      current_pos += radius * direction;
      length += radius;
    }

    pair<Vector3d, int> return_pair(ray_source, -2);
    return return_pair;
  }

}


// -2: collision not found within cut_off_length
// -1: collision is with map
pair<Vector3d, int> SkeletonFinder::raycastOnRawMap(Vector3d ray_source,
                                                    Vector3d direction,
                                                    double cut_off_length,
                                                    double search_margin) {
  // Point cloud map
  double clearance = radiusSearchOnRawMap(ray_source);
  if (clearance > cut_off_length) {
    pair<Vector3d, int> return_pair(ray_source, -2);
    return return_pair;
  } else {
    Eigen::Vector3d current_pos = ray_source + clearance * direction;
    double length = clearance;

    while (length <= cut_off_length) {
      double radius = radiusSearchOnRawMap(current_pos);

      if (radius < search_margin) {
        pair<Vector3d, int> return_pair(current_pos, -1);
        return return_pair;
      }
      current_pos += radius * direction;
      length += radius;
    }

    pair<Vector3d, int> return_pair(ray_source, -2);
    return return_pair;
  }
}


void SkeletonFinder::centralizeNodePos(NodePtr node) {
  int cnt = 0;
  Vector3d sum = Vector3d::Zero();

  vector<VertexPtr>::iterator it;
  for (it = node->black_vertices.begin(); it != node->black_vertices.end();
       it++) {
    cnt++;
    sum = sum + (*it)->coord;
  }
  node->coord = sum / cnt;
}

void SkeletonFinder::identifyBwFacets() {
  // Mesh2 is for black and white polygon
  quickhull::QuickHull<double> qh;
  quickhull::HalfEdgeMesh<double, size_t> mesh2 = qh.getConvexHullAsMesh(
      &sample_directions[0](0), sample_directions.size(), true);

  for (auto &face : mesh2.m_faces) {
    quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &halfedge =
        mesh2.m_halfEdges[face.m_halfEdgeIndex];

    vec3 vertex_quickhull;
    vector<Eigen::Vector3d> vertices_eigen;
    Eigen::Vector3d vertex_eigen;
    for (int i = 0; i < 3; i++) {
      vertex_quickhull = mesh2.m_vertices[halfedge.m_endVertex];
      vertex_eigen = Eigen::Vector3d::Zero();
      vertex_eigen << vertex_quickhull.x, vertex_quickhull.y,
          vertex_quickhull.z;
      vertices_eigen.push_back(vertex_eigen);
      halfedge = mesh2.m_halfEdges[halfedge.m_next];
    }

    bw_facets_directions.push_back(vertices_eigen);
  }
}

void SkeletonFinder::identifyFacets(NodePtr node) {
  for (vector<Eigen::Vector3d> facet_vertices : bw_facets_directions) {
    VertexPtr v1, v2, v3;
    v1 = getVertexFromDire(node, facet_vertices.at(0));
    v2 = getVertexFromDire(node, facet_vertices.at(1));
    v3 = getVertexFromDire(node, facet_vertices.at(2));
    v1->connected_vertices.push_back(v2);
    v2->connected_vertices.push_back(v3);
    v3->connected_vertices.push_back(v1);
  }
}

void SkeletonFinder::identifyFrontiers(NodePtr node) {
  // only added UNKNOWN branch in first BFS loop

  vector<vector<VertexPtr>> bv_groups;
  vector<vector<VertexPtr>> skipped_wv_groups;
  int num_wv = node->white_vertices.size();

  for (int i = 0; i < num_wv; i++) {
    VertexPtr seed_wv = node->white_vertices.at(i);
    if (seed_wv->visited)
      continue;

    seed_wv->visited = true;
    // black groups
    vector<VertexPtr> group_bv;
    // search
    deque<VertexPtr> pending_wv;
    pending_wv.push_back(seed_wv);
    // store group
    vector<VertexPtr> group_wv;
    group_wv.push_back(seed_wv);

    // for incremental exploration
    bool frontier_not_ready = false;
    while (!pending_wv.empty()) {
      VertexPtr v = pending_wv.front();
      v->visited = true;
      pending_wv.pop_front();

      for (VertexPtr v_nbhd : v->connected_vertices) {
        if (v_nbhd->type == WHITE) {
          if (v_nbhd->visited)
            continue;
          Eigen::Vector3d midpt = (v->coord + v_nbhd->coord) / 2;
          if (radiusSearch(midpt).first > 2 * _search_margin) {
            group_wv.push_back(v_nbhd);
            pending_wv.push_back(v_nbhd);
          }
        } else if (v_nbhd->type == BLACK)
            group_bv.push_back(v_nbhd);
          else if (v_nbhd->type == UNKNOWN)
            frontier_not_ready = true;
          // this branch is for incremental exploration
          // TODO: modify here so that unknown frontiers are not passed on
      }
    }
    if (frontier_not_ready) {
      skipped_wv_groups.push_back(group_wv);
      continue;
    }
    if (group_bv.size() < 3)
      continue;

    for (VertexPtr v : group_bv) {
      v->critical = true;
      v->visited = true;
    }
    bv_groups.push_back(group_bv);
  }

  // turn the visited flag off for skipped white vertices
  for (vector<VertexPtr> group_wv : skipped_wv_groups) {
    for (VertexPtr v : group_wv) {
        v->visited = false;
    }
  }

  // Filter black vertices
  int num_groups = bv_groups.size();
  for (int i = 0; i < num_groups; i++) {
    double mean_length = 0;
    double tolerance;
    for (VertexPtr v : bv_groups.at(i)) {
      mean_length += v->dis_to_center;
    }
    mean_length /= bv_groups.at(i).size();
    tolerance = mean_length * 0.3;
    int longest_index = -1;
    int shortest_index = -1;
    double longest = 0;
    double shortest = 9999;
    int num_ver = bv_groups.at(i).size();
    for (int j = 0; j < num_ver; j++) {
      if (onCeilOrFloor(bv_groups.at(i).at(j)->coord) == 0)
        continue;
      double dis = bv_groups.at(i).at(j)->dis_to_center;
      if (dis - mean_length > tolerance) {
        if (dis > longest) {
          longest_index = j;
          longest = dis;
        }
      } else if (mean_length - dis > tolerance) {
        if (dis < shortest) {
          shortest_index = j;
          shortest = dis;
        }
      }
    }
    if (longest_index != -1) {
      bv_groups.at(i).at(longest_index)->type = GREY;
      bv_groups.at(i).at(longest_index)->critical = false;
    }
    if (shortest_index != -1) {
      bv_groups.at(i).at(shortest_index)->type = GREY;
      bv_groups.at(i).at(shortest_index)->critical = false;
    }
  }

  // Inflate critical black vertices
  for (int i = 0; i < num_groups; i++) {
    for (VertexPtr v : bv_groups.at(i)) {
      for (VertexPtr v_nbhd : v->connected_vertices) {
        if (v_nbhd->type == BLACK)
          v_nbhd->critical = true;
      }
    }
  }

  // Mesh1 is for only black polygon
  vector<vec3> bv_for_mesh;
  for (VertexPtr bv : node->black_vertices) {
    if (onCeilOrFloor(bv->coord) != 0 && !bv->critical)
      continue;
    auto dir = node->sampling_directions.at(bv->sampling_dire_index);
    bv_for_mesh.push_back(dir);
    bv->critical = true;
  }

  quickhull::QuickHull<double> qh;
  quickhull::HalfEdgeMesh<double, size_t> mesh1 =
      qh.getConvexHullAsMesh(&(bv_for_mesh)[0].x, (bv_for_mesh).size(), true);

  for (auto &face : mesh1.m_faces) {
    vector<VertexPtr> vertices;

    quickhull::HalfEdgeMesh<double, size_t>::HalfEdge &halfedge =
        mesh1.m_halfEdges[face.m_halfEdgeIndex];

    for (int i = 0; i < 3; i++) {
      vec3 vertex_quickhull = mesh1.m_vertices[halfedge.m_endVertex];
      Eigen::Vector3d vertex_eigen;
      vertex_eigen << vertex_quickhull.x, vertex_quickhull.y,
          vertex_quickhull.z;
      vertices.push_back(getVertexFromDire(node, vertex_eigen));

      halfedge = mesh1.m_halfEdges[halfedge.m_next];
    }

    // Add debug checks
    if (vertices.size() != 3) {
      cout << "Warning: Invalid number of vertices for facet" << endl;
      continue;
    }
    
    for (auto v : vertices) {
      if (v == nullptr) {
        cout << "Warning: Null vertex encountered" << endl;
        continue;
      }
    }

    if (node == nullptr) {
      cout << "Warning: Null node pointer" << endl;
      continue;
    }

    FacetPtr new_facet = new Facet(vertices, node);
    int ind = node->facets.size();
    node->facets.push_back(new_facet);
    new_facet->index = ind;
  }

  // Calculate outwards normal for each facet
  int num_facet = node->facets.size();
  for (int i = 0; i < num_facet; i++) {
    FacetPtr facet_ptr = node->facets.at(i);

    Eigen::Vector3d v1 =
        facet_ptr->vertices.at(1)->coord - facet_ptr->vertices.at(0)->coord;
    Eigen::Vector3d v2 =
        facet_ptr->vertices.at(2)->coord - facet_ptr->vertices.at(0)->coord;
    Eigen::Vector3d candidate_normal = v1.cross(v2);
    candidate_normal.normalize();

    Eigen::Vector3d pt_to_judge = facet_ptr->center + candidate_normal;
    if (checkPtInPolyhedron(node, pt_to_judge))
      facet_ptr->outwards_unit_normal = -candidate_normal;
    else
      facet_ptr->outwards_unit_normal = candidate_normal;
  }

  // Create frontiers given group black vertices
  for (int i = 0; i < num_groups; i++) {
    vector<FacetPtr> group_facets =
        findGroupFacetsFromVertices(node, bv_groups.at(i));
    if (group_facets.empty())
      continue;

    findNbhdFacets(group_facets);
    vector<vector<FacetPtr>> linked_groups;
    for (FacetPtr facet : group_facets) {
      if (facet->linked)
        continue;

      vector<FacetPtr> linked_group_facets;
      deque<FacetPtr> pending_facets;
      pending_facets.push_back(facet);
      while (!pending_facets.empty()) {
        FacetPtr current_facet = pending_facets.front();
        pending_facets.pop_front();
        if (current_facet->linked)
          continue;
        linked_group_facets.push_back(current_facet);
        current_facet->linked = true;
        for (FacetPtr nbhd : current_facet->nbhd_facets) {
          if (!nbhd->linked)
            pending_facets.push_back(nbhd);
        }
      }

      linked_groups.push_back(linked_group_facets);
    }

    int num_linked_groups = linked_groups.size();
    for (int j = 0; j < num_linked_groups; j++) {
      vector<FrontierPtr> frontiers = splitFrontier(node, linked_groups.at(j));
      for (FrontierPtr f : frontiers) {
        node->frontiers.push_back(f);
      }
    }
  }

  // Add bv_group for those connecting bvs having big diff in dis_to_center
  vector<FacetPtr> jump_facets;
  for (FacetPtr facet : node->facets) {
    if (facet->valid)
      continue;
    if (facetOnCeilOrFloor(facet))
      continue;

    int count_jump = 0;
    for (int i = 0; i < 3; i++) {
      VertexPtr v1 = facet->vertices.at(i);
      VertexPtr v2 = facet->vertices.at((i + 1) % 3);
      if (v1->dis_to_center > _frontier_jump_threshold * v2->dis_to_center ||
          v2->dis_to_center > _frontier_jump_threshold * v1->dis_to_center) {
        count_jump++;
      }
    }
    if (count_jump > 1) {
      jump_facets.push_back(facet);
      facet->valid = true;
    }
  }

  findNbhdFacets(jump_facets);
  vector<vector<FacetPtr>> linked_groups;
  for (FacetPtr facet : jump_facets) {
    if (facet->linked)
      continue;

    vector<FacetPtr> linked_group_facets;
    deque<FacetPtr> pending_facets;
    pending_facets.push_back(facet);
    while (!pending_facets.empty()) {
      FacetPtr current_facet = pending_facets.front();
      pending_facets.pop_front();
      if (current_facet->linked)
        continue;
      linked_group_facets.push_back(current_facet);
      current_facet->linked = true;
      for (FacetPtr nbhd : current_facet->nbhd_facets) {
        if (!nbhd->linked)
          pending_facets.push_back(nbhd);
      }
    }

    linked_groups.push_back(linked_group_facets);
  }

  int num_linked_groups = linked_groups.size();
  for (int j = 0; j < num_linked_groups; j++) {
    FrontierPtr new_fron = new Frontier(linked_groups.at(j), node);
    if (initFrontier(new_fron))
      node->frontiers.push_back(new_fron);
  }

  // Wish to expand frontier with more facets first
  sort(node->frontiers.begin(), node->frontiers.end(), compareFrontier);
  for (FrontierPtr f : node->frontiers) {
    if (f->facets.empty())
      continue;
    pending_frontiers.push_back(f);
    int ind = loop_candidate_frontiers.size();
    f->index = ind;
    loop_candidate_frontiers.push_back(f);
  }

}

vector<FrontierPtr>
SkeletonFinder::splitFrontier(NodePtr node, vector<FacetPtr> group_facets) {
  vector<FrontierPtr> frontiers;
  // check if the group facets are too big
  if ((int)group_facets.size() <= _max_facets_grouped) {
    // find the average normal of the group facets
    Eigen::Vector3d avg_normal = Eigen::Vector3d::Zero();
    
    for (FacetPtr f : group_facets) {
      avg_normal += f->outwards_unit_normal;
    }
    avg_normal.normalize();

    // filter out outlier facets on the edge of the group
    vector<FacetPtr> filtered_group_facets;
    for (FacetPtr f : group_facets) {
      // facets on the edge of the group
      if (f->nbhd_facets.size() < 2) {
        double angle = acos(avg_normal.dot(f->outwards_unit_normal));
        if (angle > M_PI / 2.5) {
          vector<FacetPtr> single_facet;
          single_facet.push_back(f);
          FrontierPtr new_frontier = new Frontier(single_facet, node);
          frontiers.push_back(new_frontier);
          if (!initFrontier(new_frontier)) {
            cout << "Init frontier failed!" << endl;
            cout << "Change to use alternative center and normal?" << endl;
          }
          continue;
        }
      }
      filtered_group_facets.push_back(f);
    }

    FrontierPtr new_frontier = new Frontier(filtered_group_facets, node);
    frontiers.push_back(new_frontier);
    if (!initFrontier(new_frontier)) {
      cout << "Init frontier failed!" << endl;
      cout << "Change to use alternative proj_center?" << endl;
    }
  } else {
    for (FacetPtr facet : group_facets) {
      if (facet->visited)
        continue;
      facet->visited = true;

      Eigen::Vector3d normal = Eigen::Vector3d::Zero();
      vector<FacetPtr> small_group_facets;
      deque<FacetPtr> pending_facets;
      pending_facets.push_back(facet);

      while (!pending_facets.empty() &&
             (int)small_group_facets.size() < _max_facets_grouped) {
        FacetPtr f = pending_facets.front();
        pending_facets.pop_front();

        if (!small_group_facets.empty()) {
          if (acos(f->outwards_unit_normal.dot(normal)) <
              M_PI / _frontier_split_threshold) {
            normal =
                (normal * small_group_facets.size() + f->outwards_unit_normal) /
                (small_group_facets.size() + 1);
            normal.normalize();
          } else
            continue;
        } else {
          normal = f->outwards_unit_normal;
        }

        f->visited = true;
        small_group_facets.push_back(f);
        for (FacetPtr f_nbhd : f->nbhd_facets) {
          if (f_nbhd->visited)
            continue;
          pending_facets.push_back(f_nbhd);
        }
      }

      FrontierPtr new_frontier = new Frontier(small_group_facets, node);
      frontiers.push_back(new_frontier);
      if (!initFrontier(new_frontier)) {
        cout << "Init frontier failed!" << endl;
        cout << "Change to use alternative proj_center?" << endl;
      }
    }
  }
  return frontiers;
}

void SkeletonFinder::findNbhdFacets(vector<FacetPtr> facets) {
  int num_facet = facets.size();
  for (int i = 0; i < num_facet - 1; i++) {
    for (int j = i + 1; j < num_facet; j++) {
      FacetPtr f1 = facets.at(i);
      FacetPtr f2 = facets.at(j);

      if (f1->nbhd_facets.size() == 3)
        break;

      int same_vertices_count = 0;
      for (int s = 0; s < 3; s++) {
        for (int t = 0; t < 3; t++) {
          if (isSamePos(f1->vertices.at(s)->coord, f2->vertices.at(t)->coord)) {
            same_vertices_count++;
            break;
          }
        }
      }
      if (same_vertices_count == 2) {
        // if (same_vertices_count >= 1) {
        f1->nbhd_facets.push_back(f2);
        f2->nbhd_facets.push_back(f1);
      }
    }
  }
}

vector<FacetPtr>
SkeletonFinder::findGroupFacetsFromVertices(NodePtr node,
                                            vector<VertexPtr> group_bv) {
  vector<FacetPtr> group_facets;
  for (FacetPtr f : node->facets) {
    // int num_bv_included = 0;
    bool good = true;
    for (VertexPtr v_facet : f->vertices) {
      bool notIncluded = true;
      for (VertexPtr v_group : group_bv) {
        if (!v_group->critical)
          continue;
        if (isSamePos(v_facet->coord, v_group->coord)) {
          notIncluded = false;
          break;
        }
      }
      if (notIncluded) {
        good = false;
        break;
      }
      // if (!notIncluded) {
      //   num_bv_included++;
      // }
    }
    // All of the three vertices are included in the group_bv
    if (good) {
      // if (num_bv_included >= 3) {
      // Exclude facets completely on ceil or floor to improve frontier accuracy
      if ((onCeilOrFloor(f->vertices.at(0)->coord) == 1 &&
           onCeilOrFloor(f->vertices.at(1)->coord) == 1 &&
           onCeilOrFloor(f->vertices.at(2)->coord) == 1) ||
          (onCeilOrFloor(f->vertices.at(0)->coord) == -1 &&
           onCeilOrFloor(f->vertices.at(1)->coord) == -1 &&
           onCeilOrFloor(f->vertices.at(2)->coord) == -1))
        continue;
      group_facets.push_back(f);
      f->valid = true;
    }
  }
  return group_facets;
}

void SkeletonFinder::findFlowBack(NodePtr node) {
  if (node->seed_frontier == NULL)
    return;

  int size = loop_candidate_frontiers.size();
  vector<vector<Eigen::Vector3d>> flow_back_frontier_log(size);
  vector<FrontierPtr> frontiers(size);
  vector<int> pending_frontier_index;

  // Count number of contact black vertices on each frontiers
  for (VertexPtr v : node->black_vertices) {

    // only process vertices collide with other polyhedrons
    if (v->collision_node_index < 0)
      continue;

    // hit_on_pcl is on seed_frontier which is already connected
    if (checkPtOnFrontier(node->seed_frontier, v->coord))
      continue;

    FrontierPtr loop_ftr = findFlowBackFrontier(v->coord, v->collision_node_index);

    // cannot find loop frontier
    if (loop_ftr == NULL)
      continue;

    flow_back_frontier_log.at(loop_ftr->index).push_back(v->coord);
    frontiers.at(loop_ftr->index) = loop_ftr;
  }

  // Sort decreasingly: flowback frontiers with more hits first
  for (int i = 0; i < size; i++) {
    if (flow_back_frontier_log.at(i).empty())
      continue;

    int n_vertices = (int) flow_back_frontier_log.at(i).size();
    int radius_vertices = getVerticesRadius(flow_back_frontier_log.at(i));
    if (n_vertices < _min_flowback_creation_threshold 
        && radius_vertices < _min_flowback_creation_radius_threshold) 
        continue;

    if (pending_frontier_index.empty())
      pending_frontier_index.push_back(i);
    else {
      vector<int>::iterator it;
      for (it = pending_frontier_index.begin();
           it != pending_frontier_index.end(); it++) {
        if (flow_back_frontier_log.at(*it).size() <
            flow_back_frontier_log.at(i).size()) {
          pending_frontier_index.insert(it, i);
          break;
        }
        pending_frontier_index.push_back(i);
        break;
      }
    }
  }

  // Start flowback
  int size_pending_frontier = pending_frontier_index.size();
  vector<Eigen::Vector3d> connected_node_pos;
  if (node->seed_frontier->gate_node->connected_Node_ptr.empty()) {
    connected_node_pos.push_back(node->seed_frontier->master_node->coord);
  } else {
    for (NodePtr gate_con_node : node->seed_frontier->gate_node->connected_Node_ptr) {
      connected_node_pos.push_back(gate_con_node->coord);
    }
  }

  for (int i = 0; i < size_pending_frontier; i++) {
    int ind = pending_frontier_index.at(i);
    FrontierPtr flowback_frontier = frontiers.at(ind);

    // Unsafe loop: loop seg is not obstacle-free
    Eigen::Vector3d end_pt_on_frontier;
    if (flowback_frontier->gate_node == NULL)
      end_pt_on_frontier = flowback_frontier->proj_center;
    else
      end_pt_on_frontier = flowback_frontier->gate_node->coord;

    if (_exploration_mode) {
      if (!checkPathClear(node->coord, end_pt_on_frontier) ||
          !checkPathClear(flowback_frontier->master_node->coord, end_pt_on_frontier)) {
        // ROS_ERROR("Flowback seg not clear!");
        continue;
      }
    } else {
      if (!checkSegClear(node->coord, end_pt_on_frontier) ||
          !checkSegClear(flowback_frontier->master_node->coord, end_pt_on_frontier)) {
        // ROS_ERROR("Flowback seg not clear!");
        continue;
      }
    }

    // Bad loop: loop only contains 4 nodes
    if (_bad_loop) {
      bool bad_loop = false;
      for (Eigen::Vector3d pos : connected_node_pos) {
        if (flowback_frontier->gate_node == NULL ||
            flowback_frontier->gate_node->rollbacked) {
          if (isSamePos(pos, flowback_frontier->master_node->coord)) {
            bad_loop = true;
          }
        } else {
          for (NodePtr frontier_con_node :
               flowback_frontier->gate_node->connected_Node_ptr) {
            if (isSamePos(pos, frontier_con_node->coord)) {
              bad_loop = true;
              break;
            }
          }
        }
        if (bad_loop)
          break;
      }
      if (bad_loop) {
        continue;
      }
    }

    // Create flowback: new gate node if necessary
    if (flowback_frontier->gate_node == NULL) {
      NodePtr new_gate =
          new Node(flowback_frontier->proj_center, flowback_frontier, true);
      if (initNode(new_gate)) {
        flowback_frontier->gate_node = new_gate;
        flowback_frontier->master_node->connected_Node_ptr.push_back(new_gate);
        new_gate->connected_Node_ptr.push_back(flowback_frontier->master_node);
        node->connected_Node_ptr.push_back(new_gate);
        new_gate->connected_Node_ptr.push_back(node);
      } else
        continue;
    } else if (flowback_frontier->gate_node->rollbacked) {
      flowback_frontier->gate_node->rollbacked = false;
      recordNode(flowback_frontier->gate_node);
      flowback_frontier->master_node->connected_Node_ptr.push_back(
          flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(
          flowback_frontier->master_node);
      node->connected_Node_ptr.push_back(flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(node);
    } else {
      node->connected_Node_ptr.push_back(flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(node);
    }

    for (NodePtr frontier_con_node :
         flowback_frontier->gate_node->connected_Node_ptr) {
      connected_node_pos.push_back(frontier_con_node->coord);
    }
  }
}

bool SkeletonFinder::initFrontier(FrontierPtr frontier) {
  // Set proj_center
  bool proj_center_found = false;

  // Line equation:
  // x = x0 + t * nx
  // y = y0 + t * ny
  // z = z0 + t * nz
  double x0 = frontier->avg_center(0);
  double y0 = frontier->avg_center(1);
  double z0 = frontier->avg_center(2);
  double nx = frontier->outwards_unit_normal(0);
  double ny = frontier->outwards_unit_normal(1);
  double nz = frontier->outwards_unit_normal(2);

  int num_facet = frontier->facets.size();
  for (int i = 0; i < num_facet; i++) {
    double a = frontier->facets.at(i)->plane_equation(0);
    double b = frontier->facets.at(i)->plane_equation(1);
    double c = frontier->facets.at(i)->plane_equation(2);
    double d = frontier->facets.at(i)->plane_equation(3);

    double t = -(a * x0 + b * y0 + c * z0 + d) / (a * nx + b * ny + c * nz);

    Eigen::Vector3d intersection =
        frontier->avg_center + t * frontier->outwards_unit_normal;

    Eigen::Vector3d coord1 = frontier->facets.at(i)->vertices.at(0)->coord;
    Eigen::Vector3d coord2 = frontier->facets.at(i)->vertices.at(1)->coord;
    Eigen::Vector3d coord3 = frontier->facets.at(i)->vertices.at(2)->coord;

    Eigen::Vector3d cross1 = (coord2 - coord1).cross(intersection - coord1);
    Eigen::Vector3d cross2 = (coord3 - coord2).cross(intersection - coord2);
    Eigen::Vector3d cross3 = (coord1 - coord3).cross(intersection - coord3);

    if (cross1(0) * cross2(0) > 0 && cross2(0) * cross3(0) > 0 &&
        cross3(0) * cross1(0) > 0) {
      frontier->proj_center = intersection;
      frontier->proj_facet = frontier->facets.at(i);
      Eigen::Vector3d normal1 = frontier->outwards_unit_normal;
      Eigen::Vector3d normal2 = frontier->facets.at(i)->outwards_unit_normal;
      frontier->cos_theta =
          normal1.dot(normal2) / (normal1.norm() * normal2.norm());
      proj_center_found = true;
      break;
    }
  }

  if (!proj_center_found) {
    double min_angle = M_PI;
    FacetPtr best_facet;
    for (FacetPtr f : frontier->facets) {
      double angle =
          acos(frontier->outwards_unit_normal.dot(f->outwards_unit_normal));
      if (angle < min_angle) {
        min_angle = angle;
        best_facet = f;
      }
    }
    frontier->proj_facet = best_facet;
    frontier->proj_center = best_facet->center;
  }

  // Set vertices
  for (FacetPtr facet : frontier->facets) {
    for (VertexPtr v_facet : facet->vertices) {
      bool exist = false;
      if (!frontier->vertices.empty()) {
        for (VertexPtr v_frontier : frontier->vertices) {
          if (isSamePos(v_facet->coord, v_frontier->coord)) {
            exist = true;
            break;
          }
        }
      }
      if (!exist)
        frontier->vertices.push_back(v_facet);
    }
  }
  return proj_center_found;
}

bool SkeletonFinder::checkPtInPolyhedron(NodePtr node, Eigen::Vector3d pt) {
  int intersection_count = 0;
  double x = pt(0);
  double y = pt(1);
  double z = pt(2);

  int num_ftr = node->facets.size();
  for (int i = 0; i < num_ftr; i++) {
    FacetPtr inter_ftr_ptr = node->facets.at(i);

    Eigen::Vector3d coord1 = inter_ftr_ptr->vertices.at(0)->coord;
    Eigen::Vector3d coord2 = inter_ftr_ptr->vertices.at(1)->coord;
    Eigen::Vector3d coord3 = inter_ftr_ptr->vertices.at(2)->coord;

    double a = inter_ftr_ptr->plane_equation(0);
    double b = inter_ftr_ptr->plane_equation(1);
    double c = inter_ftr_ptr->plane_equation(2);
    double d = inter_ftr_ptr->plane_equation(3);

    double inter_z = -(a * x + b * y + d) / c;
    Eigen::Vector3d pt_to_judge;
    pt_to_judge << x, y, inter_z;

    if (inter_z >= z) {
      Eigen::Vector3d cross1 = (coord2 - coord1).cross(pt_to_judge - coord1);
      Eigen::Vector3d cross2 = (coord3 - coord2).cross(pt_to_judge - coord2);
      Eigen::Vector3d cross3 = (coord1 - coord3).cross(pt_to_judge - coord3);
      if (cross1(0) * cross2(0) > 0 && cross2(0) * cross3(0) > 0 &&
          cross3(0) * cross1(0) > 0) {
        intersection_count++;
      }
    }
  }

  if (intersection_count % 2 == 0)
    return false;
  else
    return true;
}

void SkeletonFinder::verifyFrontier(FrontierPtr ftr_ptr) {
  Eigen::Vector3d raycast_start_pt =
      ftr_ptr->proj_center + 2.0 * ftr_ptr->outwards_unit_normal *
                                 _search_margin;

  // check if the frontier is too near to the obstacle
  pair<double, int> rs_result = radiusSearch(raycast_start_pt);
  if (rs_result.first < _search_margin) {
    ftr_ptr->valid = false;
    return;
  }

  // raycast in outwards direction
  pair<Vector3d, int> raycast_result =
      raycast(raycast_start_pt, ftr_ptr->outwards_unit_normal,
              _max_expansion_ray_length);
  Eigen::Vector3d hit_on_pcl = raycast_result.first;

  // no hit (free space), create a new node candidate
  if (hit_on_pcl == raycast_start_pt) {
    Eigen::Vector3d new_node_candidate =
        ftr_ptr->proj_center +
        0.5 * _max_expansion_ray_length * ftr_ptr->outwards_unit_normal;
    if (checkWithinBbx(new_node_candidate)) {
      ftr_ptr->valid = true;
      ftr_ptr->next_node_pos = new_node_candidate;
    }
  }
  // normal case, create node in the midpoint
  else if (getDis(hit_on_pcl, ftr_ptr->proj_center) >
           _frontier_creation_threshold) {
    ftr_ptr->valid = true;
    ftr_ptr->next_node_pos = (hit_on_pcl + ftr_ptr->proj_center) / 2;
  }
}

void SkeletonFinder::addFacetsToPcl(NodePtr nodePtr) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr poly_pcl (new pcl::PointCloud<pcl::PointXYZ>());
  int num_facet = nodePtr->facets.size();
  for (int i = 0; i < num_facet; i++) {
    FacetPtr facet = nodePtr->facets.at(i);

    if (facet->addedToPcl)
      continue;
    facet->addedToPcl = true;

    vector<Eigen::Vector3d> start_list;
    vector<double> length_list;

    Eigen::Vector3d top_vertex = facet->vertices.at(0)->coord;
    Eigen::Vector3d left_vertex = facet->vertices.at(1)->coord;
    Eigen::Vector3d right_vertex = facet->vertices.at(2)->coord;

    Eigen::Vector3d left_to_top = top_vertex - left_vertex;
    Eigen::Vector3d left_to_top_unit = left_to_top / left_to_top.norm();
    Eigen::Vector3d left_to_right = right_vertex - left_vertex;
    Eigen::Vector3d right_to_top = top_vertex - right_vertex;
    Eigen::Vector3d right_to_top_unit = right_to_top / right_to_top.norm();
    Eigen::Vector3d right_to_left = left_vertex - right_vertex;

    double theta = acos(left_to_top.dot(left_to_right) /
                        (left_to_top.norm() * left_to_right.norm()));
    double theta_right = acos(right_to_top.dot(right_to_left) /
                              (right_to_top.norm() * right_to_left.norm()));
    double step_length = _resolution / 2;
    double start_pt_step = step_length / sin(theta);
    double end_pt_step = step_length / sin(theta_right);

    int num_start_pt = ceil(left_to_top.norm() / start_pt_step);
    double length;
    for (int j = 0; j < num_start_pt; j++) {
      Eigen::Vector3d start =
          left_vertex + j * start_pt_step * left_to_top_unit;
      Eigen::Vector3d end = right_vertex + j * end_pt_step * right_to_top_unit;
      start_list.push_back(start);
      length = (end - start).norm();
      length_list.push_back(length);
    }

    Eigen::Vector3d direction_unit = right_vertex - left_vertex;
    direction_unit.normalize();
    for (int j = 0; j < num_start_pt; j++) {
      int num_pts = ceil(length_list.at(j) / step_length);
      for (int k = 0; k < num_pts; k++) {
        Eigen::Vector3d pt_to_push =
            start_list.at(j) + k * direction_unit * step_length;
        for (int s = 0; s < 4; s++) {
          poly_pcl->points.push_back(
              pcl::PointXYZ(pt_to_push(0), pt_to_push(1), pt_to_push(2)));
          pt_to_push += (-facet->outwards_unit_normal) * step_length;
        }
      }
      Eigen::Vector3d end_to_push =
          start_list.at(j) + length_list.at(j) * direction_unit;
      for (int s = 0; s < 4; s++) {
        poly_pcl->points.push_back(
            pcl::PointXYZ(end_to_push(0), end_to_push(1), end_to_push(2)));
        end_to_push += (-facet->outwards_unit_normal) * step_length;
      }
    }
    poly_pcl->points.push_back(
        pcl::PointXYZ(top_vertex(0), top_vertex(1), top_vertex(2)));
  }

  shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> new_poly(
      new pcl::search::KdTree<pcl::PointXYZ>);

  new_poly->setInputCloud(poly_pcl);
  kdtreesForPolys.push_back(new_poly);
}
// process confirmed frontiers
bool SkeletonFinder::processFrontier(FrontierPtr curFtrPtr) {
    // Gate node: midpoint of frontier
    NodePtr gate;
    if (curFtrPtr->gate_node == NULL) {
        gate = new Node(curFtrPtr->proj_center, curFtrPtr, true);
        curFtrPtr->gate_node = gate;
    } else {
        gate = curFtrPtr->gate_node;
    }

    if (!checkWithinBbx(gate->coord) || !checkFloor(gate) ) {
        gate->rollbacked = true;
        return false;
    }

    // Center node
    NodePtr new_node = new Node(curFtrPtr->next_node_pos, curFtrPtr);
    bool init_success = initNode(new_node);
    if (init_success) {
        initNode(gate);
        curFtrPtr->master_node->connected_Node_ptr.push_back(gate);
        gate->connected_Node_ptr.push_back(curFtrPtr->master_node);
        gate->connected_Node_ptr.push_back(new_node);
        new_node->connected_Node_ptr.push_back(gate);
    } else {
        new_node->rollbacked = true;
        if (gate->connected_Node_ptr.empty()) {
            gate->rollbacked = true;
            curFtrPtr->valid = false;
            for (auto f : curFtrPtr->facets) {
                f->valid = false;
            }
        }
    }

    return init_success;
}


/* ------------------ Utility functions ----------------- */
VertexPtr SkeletonFinder::getVertexFromDire(NodePtr node,
                                            Eigen::Vector3d dire) {
  for (VertexPtr v : node->black_vertices) {
    if (isSamePos(v->dire_unit_sphere, dire))
      return v;
  }
  for (VertexPtr v : node->white_vertices) {
    if (isSamePos(v->dire_unit_sphere, dire))
      return v;
  }
  return NULL;
}

bool SkeletonFinder::checkFloor(NodePtr node) {
  Eigen::Vector3d downwards(0, 0, -1);

  pair<Vector3d, int> raycast_result =
      raycastOnRawMap(node->coord, downwards, _max_ray_length);

  if (raycast_result.second == -2) {
    // ROS_INFO("Floor not found within max_ray_length");
    return false;
  }

  double floor_height = raycast_result.first(2);


  // First node case
  if (node->seed_frontier == NULL) {
    node->dis_to_floor = floor_height;
    return true;
  }
  if (!node->isGate) {
    Eigen::Vector3d mid = (node->coord + node->seed_frontier->proj_center) / 2;
    pair<Vector3d, int> raycast_result_mid =
        raycastOnRawMap(mid, downwards, _max_ray_length);
    if (raycast_result_mid.second == -2) {
      // ROS_INFO("Floor not found within max_ray_length");
      return false;
    }

    Eigen::Vector3d mid2 = (node->seed_frontier->proj_center +
                            node->seed_frontier->master_node->coord) /
                           2;
    pair<Vector3d, int> raycast_result_mid2 =
        raycastOnRawMap(mid2, downwards, _max_ray_length);
    if (raycast_result_mid2.second == -2) {
      // ROS_INFO("Floor not found within max_ray_length");
      return false;
    }
  }

  double parent_floor_height = node->seed_frontier->master_node->dis_to_floor;

  if (fabs(floor_height - parent_floor_height) > _max_height_diff) {
    // ROS_INFO("large diff from parent");
    // ROS_INFO("parent_floor_height: %f", parent_floor_height);
    // ROS_INFO("this floor_height: %f", floor_height);
    return false;
  }

  node->dis_to_floor = floor_height;

  return true;
}

int SkeletonFinder::onCeilOrFloor(Eigen::Vector3d p) {
  // On ceil
  if (fabs(p(2) - _z_max) < _search_margin)
    return 1;
  // On floor
  if (fabs(p(2) - _z_min) < _search_margin)
    return -1;
  // Not on ceil or floor
  return 0;
}

bool SkeletonFinder::facetOnCeilOrFloor(FacetPtr f) {
  return (onCeilOrFloor(f->vertices.at(0)->coord) == -1 &&
          onCeilOrFloor(f->vertices.at(1)->coord) == -1 &&
          onCeilOrFloor(f->vertices.at(2)->coord) == -1) ||
         (onCeilOrFloor(f->vertices.at(0)->coord) == 1 &&
          onCeilOrFloor(f->vertices.at(1)->coord) == 1 &&
          onCeilOrFloor(f->vertices.at(2)->coord) == 1);
}

bool SkeletonFinder::checkSegClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2) {
  double length = (pos2 - pos1).norm();
  double step_length = _resolution;

  Eigen::Vector3d step = step_length * (pos2 - pos1) / length;
  int num_steps = ceil(length / step_length);

  Eigen::Vector3d begin_pos = pos1;
  for (int i = 0; i < num_steps; i++) {
    Eigen::Vector3d check_pos = begin_pos + i * step;
    if (collisionCheck(check_pos, _search_margin)) {
      return false;
    }
  }
  if (collisionCheck(pos2, _search_margin)) {
    return false;
  }
  return true;
}

double SkeletonFinder::getNodeRadius(NodePtr curNodePtr) {
  double node_radius = 0;
  for (VertexPtr bv : curNodePtr->black_vertices) {
    node_radius += getDis(curNodePtr->coord, bv->coord);
  }
  for (VertexPtr wv : curNodePtr->white_vertices) {
    node_radius += getDis(curNodePtr->coord, wv->coord);
  }
  node_radius = node_radius / (double)(curNodePtr->black_vertices.size() +
                                       curNodePtr->white_vertices.size());

  return node_radius;
}

double SkeletonFinder::getVerticesRadius(vector<Eigen::Vector3d> vertices) {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (Eigen::Vector3d v : vertices) {
    center += v;
  }
  center = center / (double)vertices.size();

  double radius = 0;
  for (Eigen::Vector3d v : vertices) {
    radius += getDis(center, v);
  }
  radius = radius / (double)vertices.size();

  return radius;
}

bool SkeletonFinder::checkPtOnFrontier(FrontierPtr ftr_ptr,
                                       Eigen::Vector3d pt) {
  int num_facet = ftr_ptr->facets.size();
  for (int i = 0; i < num_facet; i++) {
    FacetPtr facet = ftr_ptr->facets.at(i);
    if (checkPtOnFacet(facet, pt))
      return true;
    else
      continue;
  }
  return false;
}

bool SkeletonFinder::checkPtOnFacet(FacetPtr facet, Eigen::Vector3d pt) {
  // Take any point on the plane
  Eigen::Vector3d origin = facet->vertices.at(0)->coord;
  Eigen::Vector3d origin_to_pt = pt - origin;
  double dist_signed = origin_to_pt.dot(facet->outwards_unit_normal);

  if (fabs(dist_signed) >= 2 * _search_margin)
    return false;

  Eigen::Vector3d projected_pt = pt - dist_signed * facet->outwards_unit_normal;
  Eigen::Vector3d coord1 = facet->vertices.at(0)->coord;
  Eigen::Vector3d coord2 = facet->vertices.at(1)->coord;
  Eigen::Vector3d coord3 = facet->vertices.at(2)->coord;
  Eigen::Vector3d cross1 = (coord2 - coord1).cross(projected_pt - coord1);
  Eigen::Vector3d cross2 = (coord3 - coord2).cross(projected_pt - coord2);
  Eigen::Vector3d cross3 = (coord1 - coord3).cross(projected_pt - coord3);

  if (cross1(0) * cross2(0) > 0 && cross2(0) * cross3(0) > 0 &&
      cross3(0) * cross1(0) > 0)
    return true;
  else
    return false;
}

FrontierPtr SkeletonFinder::findFlowBackFrontier(Eigen::Vector3d pos,
                                                 int index) {

  for (FrontierPtr f : center_NodeList.at(index)->frontiers) {
    if (getDis(pos, f->proj_center) > 2 * _max_ray_length) {
      continue;
    }
      
    if (checkPtOnFrontier(f, pos))
      return f;
  }

  return NULL;
}


bool SkeletonFinder::checkWithinBbx(Eigen::Vector3d pos) {
  return pos(0) >= _x_min && pos(1) >= _y_min && pos(2) >= _z_min &&
         pos(0) <= _x_max && pos(1) <= _y_max && pos(2) <= _z_max;
}

