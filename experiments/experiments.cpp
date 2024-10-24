#include "SkeletonFinder/skeleton_finder_3D.h"
#include "SkeletonFinder/objects.h"
#include <pcl/io/pcd_io.h>

using namespace Eigen;
using namespace std;


/*
Experiments: (uses processed NodeList)

- n nodes
- n edges
- n components
- avg neighbors
- object-pairs success rate
- a* search time
- objects path length
- objects path length shortened



calculates results and saves them to a json file

*/


int getNumNodes(vector<NodePtr>& nodes) {
    return nodes.size();
}

int getNumEdges(vector<NodePtr>& nodes) {
    int num_edges = 0;
    for (int i = 0; i < num_nodes; i++) {
        num_edges += nodes.at(i)->connected_Node_ptr.size();
    }
    num_edges /= 2;
    return num_edges;
}

int getNumComponents(vector<NodePtr>& nodes) {
    vector<vector<int>> components = getDisconnectedComponents();
    int num_components = components.size();
}


struct ObjectPairExperimentResult {
    int object_id1;
    int object_id2;
    bool success;
    double path_length;
    double path_length_shortened;
    double path_find_time;
    double path_shorten_time;
};

vector<ObjectPairExperimentResult> getObjectPairStats(vector<NodePtr>& nodes, vector<MapObject>& objects) {
    vector<ObjectPairExperimentResult> results;
    for (int i = 0; i < objects.size(); i++) {
        for (int j = i + 1; j < objects.size(); j++) {
            ObjectPairExperimentResult result;
            result.object_id1 = objects.at(i).id;
            result.object_id2 = objects.at(j).id;
            // find path
            auto path_w_stats = run_findpath_w_stats(
                objects.at(i).bbox_center(0), objects.at(i).bbox_center(1), objects.at(i).bbox_center(2),
                objects.at(j).bbox_center(0), objects.at(j).bbox_center(1), objects.at(j).bbox_center(2)
            );

            vector<Eigen::Vector3d> path = path_w_stats.first;
            pair<double, double> stats = path_w_stats.second;

            result.success = path.size() > 0;
            result.path_find_time = stats.first;
            result.path_length = stats.second;

            // shorten path
            auto start = chrono::high_resolution_clock::now();
            vector<Eigen::Vector3d> new_path = shortenPath(path);
            auto finish = chrono::high_resolution_clock::now();

            result.path_shorten_time = chrono::duration_cast<chrono::duration<double>>(finish - start).count();
            result.path_length_shortened = path_finder.pathLength(new_path);

            results.push_back(result);
        }
    }
    return results;
}


void saveStats(ExperimentResult& result, const string& filename) {
    ofstream file(filename);
    json j;

    j = {
        {"num_nodes", result.num_nodes},
        {"num_edges", result.num_edges},
        {"num_components", result.num_components},
        {"mean_num_neighbors", result.mean_num_neighbors},
        {"object_pair_stats", result.object_pair_stats}
    };
    file << j.dump(4);
    file.close();
}


struct ExperimentResult {
    int num_nodes;
    int num_edges;
    int num_components;
    int mean_num_neighbors;
    vector<ObjectPairExperimentResult> object_pair_stats;
};



void SkeletonFinder::run_experiments(const string& obj_filename, const string& output_filename) {
    vector<MapObject> objects = loadObjects(obj_filename);
    vector<ObjectPairExperimentResult> object_pair_stats = getObjectPairStats(NodeList, objects);

    ExperimentResult result;
    result.num_nodes = getNumNodes(NodeList);
    result.num_edges = getNumEdges(NodeList);
    result.num_components = getNumComponents(NodeList);
    result.mean_num_neighbors = getMeanNumNeighbors(NodeList);
    result.object_pair_stats = object_pair_stats;

    saveStats(result, output_filename);

    return result;
}