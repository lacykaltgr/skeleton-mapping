#include "SkeletonFinder/skeleton_finder_3D.h"

using namespace Eigen;
using namespace std;




void SkeletonFinder::setParam(
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
) {
  _x_min = x_min;
  _x_max = x_max;
  _y_min = y_min;
  _y_max = y_max;
  _z_min = z_min;
  _z_max = z_max;
  _start_x = start_x;
  _start_y = start_y;
  _start_z = start_z;

  _map_representation = map_representation;
  _is_simulation = is_simulation;
  _frontier_creation_threshold = frontier_creation_threshold;
  _frontier_jump_threshold = frontier_jump_threshold;
  _frontier_split_threshold = frontier_split_threshold;
  _min_flowback_creation_threshold = min_flowback_creation_threshold;
  _min_flowback_creation_radius_threshold = min_flowback_creation_radius_threshold;

  _min_node_radius = min_node_radius;
  _search_margin = search_margin;
  _max_ray_length = max_ray_length;
  _max_expansion_ray_length = max_expansion_ray_length;
  _max_height_diff = max_height_diff;
  _sampling_density = sampling_density;
  _max_facets_grouped = max_facets_grouped;
  _resolution = resolution;

  _debug_mode = debug_mode;
  _bad_loop = bad_loop;

  _visualize_final_result_only = visualize_final_result_only;
  _visualize_all = visualize_all;
  _visualize_outwards_normal = visualize_outwards_normal;
  _visualize_nbhd_facets = visualize_nbhd_facets;
  _visualize_black_polygon = visualize_black_polygon;
}


