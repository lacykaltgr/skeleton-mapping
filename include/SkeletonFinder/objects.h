#ifndef OBJECTS_H
#define OBJECTS_H

#include "SkeletonFinder/skeleton_finder_3D.h"
#include <Eigen/Dense>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <nlohmann/json.hpp>

using namespace std;

// MapObject class definition
class MapObject {
public:
    string id;
    string object_tag;
    Eigen::Vector3d bbox_extent;
    Eigen::Vector3d bbox_center;
    double bbox_volume;

    MapObject(string id, const string& object_tag, const Eigen::Vector3d& bbox_extent,
              const Eigen::Vector3d& bbox_center, double bbox_volume);
};

// Function to load map objects from a JSON file
vector<MapObject> loadMapObjects(string filename) ;

// Function to initialize object frontiers
void initObjectFrontiers(vector<MapObject>& map_objects, SkeletonFinder& skeletonfinder);

#endif // OBJECTS_H
