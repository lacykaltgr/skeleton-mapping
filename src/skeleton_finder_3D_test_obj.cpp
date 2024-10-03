#include "SkeletonFinder/skeleton_finder_3D.h"
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/time.h>
#include <time.h>
#include <pcl/search/impl/kdtree.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

using namespace std;
string config_file_name;
string filename;

class MapObject {
  public:
  int id;
  string object_tag;
  Eigen::Vector3d bbox_extent;
  Eigen::Vector3d bbox_center;
  double bbox_volume;
  MapObject(int id, string object_tag, Eigen::Vector3d bbox_extent, Eigen::Vector3d bbox_center, double bbox_volume) {
    this->id = id;
    this->object_tag = object_tag;
    this->bbox_extent = bbox_extent;
    this->bbox_center = bbox_center;
    this->bbox_volume = bbox_volume;
  }
};

vector<MapObject> loadMapObjects(string filename) {
  // read json file, which is in format:
  // {"object_1": {      
  //  "id": 2,
  //  "object_tag": "sofa chair",
  //  "bbox_extent": [2.3, 1.01, 0.89],
  //  "bbox_center": [0.42, -6.29,0.52],
  //  "bbox_volume": 2.07}, 
  // "object_2": {...} }
  ifstream i(filename);
  nlohmann::json j;
  i >> j;
  vector<MapObject> map_objects;

  for (nlohmann::json::iterator it = j.begin(); it != j.end(); ++it) {
    string object_name = it.key();
    nlohmann::json object = it.value();
    int id = object["id"];
    string object_tag = object["object_tag"];
    Eigen::Vector3d bbox_extent = Eigen::Vector3d(object["bbox_extent"][0], object["bbox_extent"][1], object["bbox_extent"][2]);
    Eigen::Vector3d bbox_center = Eigen::Vector3d(object["bbox_center"][0], object["bbox_center"][1], object["bbox_center"][2]);
    double bbox_volume = object["bbox_volume"];
    MapObject map_object(id, object_tag, bbox_extent, bbox_center, bbox_volume);
    map_objects.push_back(map_object);
  }

  return map_objects;
}

void initObjectFrontiers(vector<MapObject>& map_objects, SkeletonFinder& skeletonfinder) {
  for (int i = 0; i < map_objects.size(); i++) {

    MapObject map_object = map_objects[i];
    Eigen::Vector3d bbox_center = map_object.bbox_center;
    Eigen::Vector3d bbox_extent = map_object.bbox_extent;
    Eigen::Vector3d bbox_min = bbox_center - bbox_extent;
    Eigen::Vector3d bbox_max = bbox_center + bbox_extent;


    NodePtr master_node = new Node(bbox_center, NULL, false);
    vector<string> frontier_tags = {"x_min", "x_max", "y_min", "y_max", "z_max"};

    Eigen::Vector3d a, b, c, d;
    Eigen::Vector3d a_dir, b_dir, c_dir, d_dir;
    Eigen::Vector3d middle, middle_direction;

    for (string tag: frontier_tags) {
      
      if (tag == "x_min") { // left

        a = Eigen::Vector3d(bbox_min(0), bbox_min(1), bbox_max(2));
        b = Eigen::Vector3d(bbox_min(0), bbox_max(1), bbox_max(2));
        c = Eigen::Vector3d(bbox_min(0), bbox_max(1), bbox_min(2));
        d = Eigen::Vector3d(bbox_min(0), bbox_min(1), bbox_min(2));
        middle = Eigen::Vector3d(bbox_min(0), bbox_center(1), bbox_center(2));

      } else if (tag == "x_max") { // right
        a = Eigen::Vector3d(bbox_max(0), bbox_min(1), bbox_min(2));
        b = Eigen::Vector3d(bbox_max(0), bbox_max(1), bbox_min(2));
        c = Eigen::Vector3d(bbox_max(0), bbox_max(1), bbox_max(2));
        d = Eigen::Vector3d(bbox_max(0), bbox_min(1), bbox_max(2));
        middle = Eigen::Vector3d(bbox_max(0), bbox_center(1), bbox_center(2));
          
        } else if (tag == "y_min") { //front

        a = Eigen::Vector3d(bbox_min(0), bbox_min(1), bbox_min(2));
        b = Eigen::Vector3d(bbox_max(0), bbox_min(1), bbox_min(2));
        c = Eigen::Vector3d(bbox_max(0), bbox_min(1), bbox_max(2));
        d = Eigen::Vector3d(bbox_min(0), bbox_min(1), bbox_max(2));
        middle = Eigen::Vector3d(bbox_center(0), bbox_center(1), bbox_min(2));
  
        } else if (tag == "y_max") { // back

        a = Eigen::Vector3d(bbox_min(0), bbox_max(1), bbox_min(2));
        b = Eigen::Vector3d(bbox_max(0), bbox_max(1), bbox_min(2));
        c = Eigen::Vector3d(bbox_max(0), bbox_max(1), bbox_max(2));
        d = Eigen::Vector3d(bbox_min(0), bbox_max(1), bbox_max(2));
        middle = Eigen::Vector3d(bbox_center(0), bbox_center(1), bbox_max(2));
  
        } else if (tag == "z_max") { // top

        a = Eigen::Vector3d(bbox_min(0), bbox_min(1), bbox_max(2));
        b = Eigen::Vector3d(bbox_max(0), bbox_min(1), bbox_max(2));
        c = Eigen::Vector3d(bbox_max(0), bbox_max(1), bbox_max(2));
        d = Eigen::Vector3d(bbox_min(0), bbox_max(1), bbox_max(2));
        middle = Eigen::Vector3d(bbox_center(0), bbox_max(1), bbox_center(2));

      }

      a_dir = (a - bbox_center) / (a - bbox_center).norm();
      b_dir = (b - bbox_center) / (b - bbox_center).norm();
      c_dir = (c - bbox_center) / (c - bbox_center).norm();
      d_dir = (d - bbox_center) / (d - bbox_center).norm();

      VertexPtr a_vertex = new Vertex(a, a_dir, BLACK);
      VertexPtr b_vertex = new Vertex(b, b_dir, BLACK);
      VertexPtr c_vertex = new Vertex(c, c_dir, BLACK);
      VertexPtr d_vertex = new Vertex(d, d_dir, BLACK);

      vector<VertexPtr> vertices_1 = {a_vertex, b_vertex, c_vertex};
      vector<VertexPtr> vertices_2 = {c_vertex, d_vertex, a_vertex};

      FacetPtr facet_1 = new Facet(vertices_1, master_node);
      FacetPtr facet_2 = new Facet(vertices_2, master_node);

      middle_direction = (middle - bbox_center) / (middle - bbox_center).norm();
      facet_1->outwards_unit_normal = middle_direction;
      facet_2->outwards_unit_normal = middle_direction;

      master_node->facets.push_back(facet_1);
      master_node->facets.push_back(facet_2);

      FrontierPtr frontier = new Frontier({facet_1, facet_2}, master_node);
      frontier->proj_center = middle;
      frontier->proj_facet = facet_1;
      frontier->vertices.push_back(a_vertex);
      frontier->vertices.push_back(b_vertex);
      frontier->vertices.push_back(c_vertex);
      frontier->vertices.push_back(d_vertex);

      master_node->frontiers.push_back(frontier);

      skeletonfinder.addInitialFrontier(frontier);
    }

    skeletonfinder.recordNode(master_node);
    skeletonfinder.addFacetsToPcl(master_node);
  }

}




int main(int argc, char **argv) {
  filename = argv[1];
  if (argc < 3) {
    config_file_name = "/app/graph/3D_Sparse_Skeleton/3d_sparse_skeleton/polygon_generation/config.yaml";
  } else {
    config_file_name = argv[2];
  }

  YAML::Node config = YAML::LoadFile(config_file_name);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PCDReader reader;
  reader.read(filename, *cloud);

  cout << "Map successfully loaded..." << endl;
  cout << "Size of map: " << (*cloud).points.size() << endl;

  SkeletonFinder skeleton_finder_3D(config);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  vector<MapObject> map_objects = loadMapObjects("/workspace/ros2_ws/src/global_planner/libraries/polygon_generation/resource/obj_json_mapping.json");
  initObjectFrontiers(map_objects, skeleton_finder_3D);

  skeleton_finder_3D.run_processing(cloud_xyz);

  double path_start_x = config["path_start"]["x"].as<double>();
  double path_start_y = config["path_start"]["y"].as<double>();
  double path_start_z = config["path_start"]["z"].as<double>();
  double path_target_x = config["path_target"]["x"].as<double>();
  double path_target_y = config["path_target"]["y"].as<double>();
  double path_target_z = config["path_target"]["z"].as<double>();
  skeleton_finder_3D.run_findpath(
    path_start_x, path_start_y, path_start_z,
    path_target_x, path_target_y, path_target_z
  );
}