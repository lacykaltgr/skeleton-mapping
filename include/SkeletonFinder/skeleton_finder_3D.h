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
  // Map representation
  // 0: point cloud; 1: occupancy map
  int _map_representation;
  // Whether the map is in simulation
  bool _is_simulation;
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
  // Visualization
  double _truncated_z_high;
  double _truncated_z_low;
  // Bounding box
  double _x_min, _x_max, _y_min, _y_max, _z_min, _z_max;
  double _start_x, _start_y, _start_z;

  /* ------------------ Development Tune ------------------ */
  bool _debug_mode;
  bool _bad_loop;

  // Visualize only the final result or the expansion process
  bool _visualize_final_result_only;
  // Visualize all or only the newest polyhedron
  bool _visualize_all;
  // Visualize outwards normal for each frontier
  bool _visualize_outwards_normal;
  // Visualize neighborhood facets for each frontier
  bool _visualize_nbhd_facets;
  // Visualize only_black polygon or black_and_white polygon
  bool _visualize_black_polygon;

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

  /* -------------------- Sampling Use -------------------- */
  random_device rd;
  default_random_engine eng;

  // random distribution for generating samples inside a unit circle
  uniform_real_distribution<double> rand_rho = uniform_real_distribution<double>(0.0, 1.0);
  uniform_real_distribution<double> rand_phi = uniform_real_distribution<double>(0.0, 2 * M_PI);

  // basic random distributions for generating samples, in all feasible regions
  uniform_real_distribution<double> rand_x, rand_y, rand_z, rand_bias;
  // random distribution, especially for generating samples inside the local
  // map's boundary
  uniform_real_distribution<double> rand_x_in, rand_y_in, rand_z_in;

 public:
  SkeletonFinder(YAML::Node &config);
  ~SkeletonFinder();

  /* set-up functions */
  void reset();
  void setParam(
    double x_min, double x_max, double y_min, double y_max, double z_min, double z_max,
    double start_x, double start_y, double start_z,
    int map_representation, bool is_simulation,
    double frontier_creation_threshold, double frontier_jump_threshold, double frontier_split_threshold,
    int min_flowback_creation_threshold, double min_flowback_creation_radius_threshold,
    double min_node_radius, double search_margin, double max_ray_length,
    double max_expansion_ray_length, double max_height_diff, int sampling_density,
    int max_facets_grouped, double resolution,
    bool debug_mode, bool bad_loop,
    bool visualize_final_result_only, bool visualize_all, bool visualize_outwards_normal,
    bool visualize_nbhd_facets, bool visualize_black_polygon
  );
  void setStartPt(Eigen::Vector3d startPt);
  void saveToFile(const std::string& filename, const std::string& data);

  /* main function entries */
  void skeletonExpansion(Eigen::Vector3d startPt);
  bool initNode(NodePtr curNodePtr);
  void checkFacetClearance(NodePtr curNodePtr);
  bool checkPosConnected(NodePtr node_ptr, Eigen::Vector3d pos);
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
  void combineFrontier(NodePtr node);
  bool initFrontier(FrontierPtr frontier);
  bool checkFacetsCombine(FacetPtr f1, FacetPtr f2);
  bool checkPtInPolyhedron(NodePtr node, Eigen::Vector3d pt);
  void verifyFrontier(FrontierPtr ftr_ptr);
  void addFacetsToPcl(NodePtr nodePtr);
  bool processFrontier(FrontierPtr curFtrPtr);
  vector<Eigen::Vector3d> findPath(Eigen::Vector3d start, Eigen::Vector3d target);
  vector<Eigen::Vector3d> findPathByAStar(Eigen::Vector3d start, Eigen::Vector3d target);

  bool isSamePos(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  bool checkFloor(NodePtr node);
  int onCeilOrFloor(Eigen::Vector3d p);
  bool facetOnCeilOrFloor(FacetPtr f);
  bool checkSegClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  double getNodeRadius(NodePtr curNodePtr);
  double getVerticesRadius(vector<Eigen::Vector3d> vertices);
  VertexPtr getVertexFromDire(NodePtr node, Eigen::Vector3d dire);
  FacetPtr getCommonInvalidNbhdFacet(FrontierPtr f1, FrontierPtr f2, Eigen::Vector3d pt);
  vector<Eigen::Vector3d> getCommonVertices(FrontierPtr f1, FrontierPtr f2);
  bool checkPtOnFrontier(FrontierPtr ftr_ptr, Eigen::Vector3d pt);
  bool checkPtOnFacet(FacetPtr facet, Eigen::Vector3d pt);
  FrontierPtr findFlowBackFrontier(Eigen::Vector3d hit_on_pcl, int index);
  FacetPtr findFlowBackFacet(Eigen::Vector3d pos, int index);
  bool checkWithinBbx(Eigen::Vector3d pos);
  void addBbxToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double leaf);
  void findBoundingBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr map);

  void run_processing(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_map);  // Changed parameter type
  pair<vector<Eigen::Vector3d>, vector<double>> run_findpath(double _path_start_x, double _path_start_y, double _path_start_z,
                      double _path_target_x, double _path_target_y, double _path_target_z);
  pair<vector<Eigen::Vector3d>, vector<double>> run_findpath2(double _path_start_x, double _path_start_y, double _path_start_z,
                      double _path_target_x, double _path_target_y, double _path_target_z);             
  void run_postprocessing(double base_height, double connectionRadius, double tooCloseThreshold);
  void addInitialFrontier(FrontierPtr frontier);
  bool canBeReplacedBy(NodePtr node_to_keep, NodePtr node_to_remove);
  bool checkPathClear(Eigen::Vector3d pos1, Eigen::Vector3d pos2);
  bool isValidPosition(Eigen::Vector3d base_pos);
  void handleInterSections(vector<NodePtr> validNodeList, double base_height);
  bool calculateIntersection2D(const Eigen::Vector2d& p1_start, const Eigen::Vector2d& p1_end,
                             const Eigen::Vector2d& p2_start, const Eigen::Vector2d& p2_end,
                             Eigen::Vector2d& intersection);
  void removeTooCloseNodes(vector<TooCloseCandidate> tooCloseCandidates, vector<NodePtr>& validNodeList);
  void mergeNodes(NodePtr node1, NodePtr node2, vector<NodePtr>& validNodeList, double node1_w, double node2_w);
  void killOtherNode(NodePtr node_to_keep, NodePtr node_to_remove, vector<NodePtr>& validNodeList);
  vector<NodePtr> closestNodes(NodePtr node, double maxDistance, vector<NodePtr>& validNodeList);
  Eigen::MatrixXd getAdjMatrix(vector<NodePtr> validNodeList);
  vector<NodePtr> pathToNodes(vector<Eigen::Vector3d> path);
  double calculateSafeRadius(NodePtr node, NodePtr connected_node);
  vector<double> pathRadiuses(vector<Eigen::Vector3d> path);
  


  /* operations on the tree */
  void recordNode(NodePtr new_node);


  vector<vector<int>> spectralClustering(vector<NodePtr> validNodeList);
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
  inline double radiusSearchOnVisMap(Eigen::Vector3d &pt);
  bool collisionCheck(Eigen::Vector3d search_pt, double threshold);

  // inline NodePtr genNewNode(Eigen::Vector3d &pt);
  // NodePtr pending_nodes_pop_front() {
  //   NodePtr curNodePtr = pending_nodes.front();
  //   pending_nodes.pop_front();
  //   return curNodePtr;
  // };
  FrontierPtr pendingFrontiersPopFront() {
    FrontierPtr curFrontierPtr = pending_frontiers.front();
    pending_frontiers.pop_front();
    return curFrontierPtr;
  };

  /* -------------------- visualization ------------------- */
  void visualization();
  void visNodesAndVertices();
  void visMap();
  void visPolygons();
  void visFrontiers();
  void visCurrentFrontier(FrontierPtr ftr);
  void visConnections();
  void visSphere(Eigen::Vector3d pos, double radius);
  void visPath();
  void visStart();
};