#ifndef SKELETON_FINDER_3D_H
#define SKELETON_FINDER_3D_H

#include <math.h>
#include <Eigen/Eigen>
#include <deque>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>

#include "SkeletonFinder/quickhull/QuickHull.hpp"
#include "SkeletonFinder/A_star.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include "SkeletonFinder/backward.hpp"
#include "SkeletonFinder/data_type_3D.h"
#include <yaml-cpp/yaml.h>
#include <map>

inline bool compareFrontier(FrontierPtr f1, FrontierPtr f2) {
  return f1->facets.size() > f2->facets.size();
}


class SkeletonFinder {
 private:
  vector<shared_ptr<pcl::search::KdTree<pcl::PointXYZ>>> kdtreesForPolys;
  pcl::search::KdTree<pcl::PointXYZ> kdtreeForRawMap;
  pcl::search::KdTree<pcl::PointXYZ> kdtreeForNodes;

  a_star::AStar path_finder;

  // deque<NodePtr> pending_nodes;
  deque<FrontierPtr> pending_frontiers;
  deque<FrontierPtr> loop_candidate_frontiers;
  vector<NodePtr> NodeList;
  vector<NodePtr> center_NodeList;
  vector<Eigen::Vector3d> sample_directions;
  vector<vector<Eigen::Vector3d>> bw_facets_directions;

  vector<int> pointIdxRadiusSearchForRawMap;
  vector<float> pointRadiusSquaredDistanceForRawMap;
  vector<int> pointIdxRadiusSearchForNodes;
  vector<float> pointRadiusSquaredDistanceForNodes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr nodes_pcl;

  /* ------------------------ Param ----------------------- */
  // An edge will be considered as a frontier if:
  // the dist to its nearest point exceeds this threshold
  double _frontier_creation_threshold;
  // Jump frontier
  double _frontier_jump_threshold;
  // Facets will be split into diff frontier if the angle between exceeds this threshold
  double _frontier_split_threshold;
  // A flowback will be created if number of contact vertices exceeds this threshold
  int _min_flowback_creation_threshold;
  // A flowback will not be created if the radius of contact vertices is below this threshold
  double _min_flowback_creation_radius_threshold;
  // A node will be discarded if its average vertex-center distance is below
  // this threshold
  double _min_node_radius;
  // A point on the ray will be considered as hit the pcl if:
  // the dist to its nearest point is below this margin
  // search_margin > sqrt((resolution/2)^2 + (raycast_step/2)^2)
  double _search_margin;
  // A ray will be discarded if length exceeds this max
  double _max_ray_length;
  // A new node will be set at the midpoint if length exceeds this max
  double _max_expansion_ray_length;
  // A node will be absorbed if difference of distance to floor with its parent exceeds this limit
  double _max_height_diff;
  // Number of sampings on the unit sphere
  int _sampling_density;
  // Max number of facets grouped in a frontier
  int _max_facets_grouped;
  // Resolution for map, raycast,
  double _resolution;
  // Bounding box
  double _x_min, _x_max, _y_min, _y_max, _z_min, _z_max;
  double _start_x, _start_y, _start_z;


  // Base height and radius
  double _base_height, _base_radius;
  // Connection radius
  double _connection_radius;
  // Too close threshold
  double _too_close_threshold;

  // Robot type 0:ground 1:aerial
  int _robot_type;

  // Raywalking max height diff
  double _raywalking_max_height_diff;

  // Exploration mode
  bool _exploration_mode;


  /* ------------------ Development Tune ------------------ */
  bool _bad_loop;

  // Timers
  double genBlackAndWhiteVertices_timing = 0;
  double convex_hull_timing = 0;
  double centralizeNodePos_timing = 0;
  double findFlowBack_timing = 0;
  double identifyFacets_timing = 0;
  double identifyFrontiers_timing = 0;
  double addFacetsToPcl_timing = 0;
  double verifyFrontier_timing = 0;
  double processFrontier_timing = 0;


 public:
  SkeletonFinder(YAML::Node &config);
  ~SkeletonFinder();

  /* set-up functions */
  void reset();
  void setParam(
    double x_min, double x_max, double y_min, double y_max, double z_min, double z_max,
    double start_x, double start_y, double start_z,
    double frontier_creation_threshold, double frontier_jump_threshold, double frontier_split_threshold,
    int min_flowback_creation_threshold, double min_flowback_creation_radius_threshold,
    double min_node_radius, double search_margin, double max_ray_length,
    double max_expansion_ray_length, double max_height_diff, int sampling_density,
    int max_facets_grouped, double resolution, bool bad_loop,
    double base_height, double base_radius, double connection_radius, double too_close_threshold,
    int robot_type, double raywalking_max_height_diff, bool exploration_mode
  );
  void setStartPt(Eigen::Vector3d startPt);

  /* main function entries */
  void skeletonExpansion(Eigen::Vector3d startPt);
  bool initNode(NodePtr curNodePtr);
  void genBlackAndWhiteVertices(NodePtr nodePtr);
  void genSamplesOnUnitSphere();
  void genSamplesOnUnitCircle();
  pair<Eigen::Vector3d, int> raycast(Eigen::Vector3d ray_source, Eigen::Vector3d direction,
                                     double cut_off_length);
  pair<Eigen::Vector3d, int> raycastOnRawMap(Eigen::Vector3d ray_source, Eigen::Vector3d direction,
                                             double cut_off_length);
  pair<Eigen::Vector3d, int> raycastOnRawMap(Eigen::Vector3d ray_source, Eigen::Vector3d direction,
                                            double cut_off_length, double search_margin);
  void centralizeNodePos(NodePtr node);
  void identifyBwFacets();
  void identifyFacets(NodePtr node);
  void findNbhdFacets(vector<FacetPtr> facets);
  void identifyFrontiers(NodePtr node);
  vector<FrontierPtr> splitFrontier(NodePtr node, vector<FacetPtr> group_facets);
  vector<FacetPtr> findGroupFacetsFromVertices(NodePtr node, vector<VertexPtr> group_bv);
  void findFlowBack(NodePtr node);
  bool initFrontier(FrontierPtr frontier);
  bool checkPtInPolyhedron(NodePtr node, Eigen::Vector3d pt);
  void verifyFrontier(FrontierPtr ftr_ptr);
  void addFacetsToPcl(NodePtr nodePtr);
  bool processFrontier(FrontierPtr curFtrPtr);
  vector<Eigen::Vector3d> findPath(Eigen::Vector3d start, Eigen::Vector3d target);
  vector<Eigen::Vector3d> findPathByAStar(Eigen::Vector3d start, Eigen::Vector3d target);
  vector<Eigen::Vector3d> findPathBetweenObjects(Eigen::Vector3d start, Eigen::Vector3d target,
                                                 Eigen::Vector3d start_bbox_extent, Eigen::Vector3d start_box_center,
                                                 Eigen::Vector3d target_bbox_extent, Eigen::Vector3d target_box_center);

  bool isSamePos(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  bool checkFloor(NodePtr node);
  int onCeilOrFloor(Eigen::Vector3d p);
  bool facetOnCeilOrFloor(FacetPtr f);
  bool checkSegClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  double getNodeRadius(NodePtr curNodePtr);
  double getVerticesRadius(vector<Eigen::Vector3d> vertices);
  VertexPtr getVertexFromDire(NodePtr node, Eigen::Vector3d dire);
  bool checkPtOnFrontier(FrontierPtr ftr_ptr, Eigen::Vector3d pt);
  bool checkPtOnFacet(FacetPtr facet, Eigen::Vector3d pt);
  FrontierPtr findFlowBackFrontier(Eigen::Vector3d hit_on_pcl, int index);
  bool checkWithinBbx(Eigen::Vector3d pos);
  void addBbxToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double leaf);
  void findBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);

  void run_processing(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_map);  // Changed parameter type
  void run_postprocess_edges();
  pair<vector<Eigen::Vector3d>, pair<double, double>> run_findpath_w_stats(
    double _path_start_x, double _path_start_y, double _path_start_z,
    double _path_target_x, double _path_target_y, double _path_target_z
  );        
  pair<vector<Eigen::Vector3d>, vector<double>> run_findpath(
    double _path_start_x, double _path_start_y, double _path_start_z,
    double _path_target_x, double _path_target_y, double _path_target_z
  );
  pair<vector<Eigen::Vector3d>, vector<double>> run_findpath_shorten(
    double _path_start_x, double _path_start_y, double _path_start_z,
    double _path_target_x, double _path_target_y, double _path_target_z
  );
  vector<Eigen::Vector3d> shortenPath(Eigen::Vector3d start, vector<Eigen::Vector3d> path);
  bool rayFlying(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius);
  bool rayWalking(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double max_step_height);
  double checkFloorHeight(Eigen::Vector3d base_pos, double base_radius, double cut_off_length);
  void fillNodesPcl(double base_height, double base_radius);
  int drawEdgesRawWalking(vector<NodePtr>& validNodeList, double connectionRadius);
  void run_postprocessing();
  void addInitialFrontier(FrontierPtr frontier);
  bool canBeReplacedBy(NodePtr node_to_keep, NodePtr node_to_remove);
  bool checkPathClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  double checkPathClearLength(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  double rayFlyingLen(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius);
  double rayWalkingLen(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double max_step_height);
  bool canRayFly(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius);
  bool canRayWalk(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double base_radius, double max_step_height);
  bool isValidPosition(Eigen::Vector3d base_pos);
  int removeTooCloseNodes(double tooCloseThreshold);
  void mergeNodes(NodePtr node1, NodePtr node2, vector<NodePtr>& validNodeList, double node1_w, double node2_w);
  void killOtherNode(NodePtr node_to_keep, NodePtr node_to_remove, vector<NodePtr>& validNodeList);
  vector<NodePtr> closestNodes(NodePtr node, double maxDistance, vector<NodePtr>& validNodeList);
  Eigen::MatrixXd getAdjMatrix(vector<NodePtr> validNodeList);
  vector<NodePtr> pathToNodes(vector<Eigen::Vector3d> path);
  double calculateSafeRadius(NodePtr node, NodePtr connected_node);
  vector<double> pathRadiuses(vector<Eigen::Vector3d> path);

  vector<vector<int>> getDisconnectedComponents(vector<NodePtr> nodes);
  


  /* operations on the tree */
  void recordNode(NodePtr new_node);

  void save_clusters(vector<vector<int>> clusters, vector<NodePtr> validNodeList);


  /* utility functions */
  inline double getDis(const NodePtr node1, const NodePtr node2) {
    return sqrt(pow(node1->coord(0) - node2->coord(0), 2) +
                pow(node1->coord(1) - node2->coord(1), 2) +
                pow(node1->coord(2) - node2->coord(2), 2));
  }

  inline double getDis(const NodePtr node1, const Eigen::Vector3d &pt) {
    return sqrt(pow(node1->coord(0) - pt(0), 2) +
                pow(node1->coord(1) - pt(1), 2) +
                pow(node1->coord(2) - pt(2), 2));
  }

  inline double getDis(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
    return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2) +
                pow(p1(2) - p2(2), 2));
  }

  inline double getDis(const Eigen::Vector3i &p1, const Eigen::Vector3i &p2) {
    return sqrt((double)(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2) +
                        pow(p1(2) - p2(2), 2)));
  }

  // inline Eigen::Vector3d genSample();
  inline pair<double, int> radiusSearch(Eigen::Vector3d &pt);
  inline double radiusSearchOnRawMap(Eigen::Vector3d &pt);
  bool collisionCheck(Eigen::Vector3d search_pt, double threshold);

  FrontierPtr pendingFrontiersPopFront() {
    FrontierPtr curFrontierPtr = pending_frontiers.front();
    pending_frontiers.pop_front();
    return curFrontierPtr;
  };

  vector<NodePtr> getNodes() { return NodeList; };
  a_star::AStar getPathFinder() { return path_finder; };

  // save

  void saveNodes(const std::string& filename);
  void saveConnections(const std::string& filename);
  void saveAdjMatrix(const std::string& filename);
  void loadNodes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void loadConnections(const std::string& filename);


  // baselines
  void run_processing_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size);
  void run_processing_random(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, int num_points);
  void run_processing_grid_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size, double base_height);
  void run_processing_grid_3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, double cell_size);
  void run_processing_random_2d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, int num_points, double base_height, double base_radius);
  void run_processing_random_3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_pcl, int num_points, double base_radius);



  // generate json for config

 std::map<std::string, double> getConfigMap() {
    std::map<std::string, double> config_map;
    config_map["x_min"] = _x_min;
    config_map["x_max"] = _x_max;
    config_map["y_min"] = _y_min;
    config_map["y_max"] = _y_max;
    config_map["z_min"] = _z_min;
    config_map["z_max"] = _z_max;
    config_map["start_x"] = _start_x;
    config_map["start_y"] = _start_y;
    config_map["start_z"] = _start_z;
    config_map["min_node_radius"] = _min_node_radius;
    config_map["search_margin"] = _search_margin;
    config_map["max_ray_length"] = _max_ray_length;
    config_map["max_expansion_ray_length"] = _max_expansion_ray_length;
    config_map["max_height_diff"] = _max_height_diff;
    config_map["sampling_density"] = _sampling_density;
    config_map["max_facets_grouped"] = _max_facets_grouped;
    config_map["resolution"] = _resolution;
    config_map["base_height"] = _base_height;
    config_map["base_radius"] = _base_radius;
    config_map["connection_radius"] = _connection_radius;
    config_map["too_close_threshold"] = _too_close_threshold;
    config_map["robot_type"] = static_cast<double>(_robot_type);  // Convert int to double
    config_map["raywalking_max_height_diff"] = _raywalking_max_height_diff;
    config_map["exploration_mode"] = static_cast<double>(_exploration_mode);  // Convert bool to double
    config_map["frontier_creation_threshold"] = _frontier_creation_threshold;
    config_map["frontier_jump_threshold"] = _frontier_jump_threshold;
    config_map["frontier_split_threshold"] = _frontier_split_threshold;
    config_map["min_flowback_creation_threshold"] = _min_flowback_creation_threshold;
    config_map["min_flowback_creation_radius_threshold"] = _min_flowback_creation_radius_threshold;
    return config_map;
}

  
};

#endif