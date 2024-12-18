#include "SkeletonFinder/skeleton_finder_3D.h"
#include "SkeletonFinder/experiments.h"
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



int SkeletonFinderExperiment::getNumNodes(SkeletonFinder skeletonfinder) {
    return skeletonfinder.getNodes().size();
}

int SkeletonFinderExperiment::getNumEdges(SkeletonFinder skeletonfinder) {
    vector<NodePtr> nodes = skeletonfinder.getNodes();
    int num_nodes = nodes.size();
    int num_edges = 0;
    for (int i = 0; i < num_nodes; i++) {
        num_edges += nodes.at(i)->connected_Node_ptr.size();
    }
    num_edges /= 2;
    return num_edges;
}

int SkeletonFinderExperiment::getNumComponents(SkeletonFinder skeletonfinder) {
    vector<vector<int>> components = skeletonfinder.getDisconnectedComponents(skeletonfinder.getNodes());
    int num_components = components.size();
    return num_components;
}


vector<ObjectPairExperimentResult> SkeletonFinderExperiment::getObjectPairStats(SkeletonFinder skeletonfinder, vector<MapObject>& objects) {
    vector<ObjectPairExperimentResult> results;
    vector<Eigen::Vector3d> seen_objects;

    for (size_t i = 0; i < objects.size(); i++) {

        show_progress_bar((float)i / objects.size());
        
        for (Eigen::Vector3d seen_object : seen_objects)
            if ((objects.at(i).bbox_center - seen_object).norm() < 1)
                continue;
        seen_objects.push_back(Eigen::Vector3d(objects.at(i).bbox_center(0), objects.at(i).bbox_center(1), objects.at(i).bbox_center(2)));

        for (size_t j = i + 1; j < objects.size(); j++) {

            Eigen::Vector3d start_query(objects.at(i).bbox_center(0), objects.at(i).bbox_center(1), objects.at(i).bbox_center(2));
            Eigen::Vector3d target_query(objects.at(j).bbox_center(0), objects.at(j).bbox_center(1), objects.at(j).bbox_center(2));

            // check if there is a minimum distance between the objects
            double min_distance = 10;
            if ((start_query - target_query).norm() < min_distance)
                continue;

            auto begin = chrono::high_resolution_clock::now();
            vector<Eigen::Vector3d> path = skeletonfinder.findPathBetweenObjects(
                start_query, target_query, 
                objects.at(i).bbox_extent, objects.at(i).bbox_center, 
                objects.at(j).bbox_extent, objects.at(j).bbox_center
            );
            auto finish = chrono::high_resolution_clock::now();


            ObjectPairExperimentResult result;
            result.object_id1 = objects.at(i).id;
            result.object_id2 = objects.at(j).id;
            result.success = path.size() > 0;

            if (result.success) {
                result.euclidean_distance = (start_query - target_query).norm();

                result.path_find_time = chrono::duration_cast<chrono::duration<double>>(finish - begin).count();
                result.path_length = skeletonfinder.getPathFinder().pathLength(path);

                // shorten path
                Eigen::Vector3d start_o(objects.at(i).bbox_center(0), objects.at(i).bbox_center(1), objects.at(i).bbox_center(2));
                auto start = chrono::high_resolution_clock::now();
                vector<Eigen::Vector3d> new_path = skeletonfinder.shortenPath(start_o, path);
                auto finish = chrono::high_resolution_clock::now();

                result.path_shorten_time = chrono::duration_cast<chrono::duration<double>>(finish - start).count();
                result.path_length_shortened = skeletonfinder.getPathFinder().pathLength(new_path);
            } else {
                result.path_find_time = 0;
                result.path_length = 0;
                result.path_shorten_time = 0;
                result.path_length_shortened = 0;
            }

            results.push_back(result);
        }
    }

    return results;
}




ExperimentResult SkeletonFinderExperiment::run_experiments(SkeletonFinder skeletonfinder, const string& obj_filename) {
    vector<MapObject> objects = loadMapObjects(obj_filename);
    vector<ObjectPairExperimentResult> object_pair_stats = getObjectPairStats(skeletonfinder, objects);

    ExperimentResult result;
    result.num_nodes = getNumNodes(skeletonfinder);
    result.num_edges = getNumEdges(skeletonfinder);
    result.mean_num_neighbors = result.num_edges * 2 / result.num_nodes;
    result.object_pair_stats = object_pair_stats;

    cout << "Number of nodes: " << result.num_nodes << endl;

    int success_count = 0;
    double total_path_find_time = 0;
    double total_path_length = 0;
    double total_path_shorten_time = 0;
    double total_path_length_shortened = 0;
    double total_path_find_time_normalized = 0;
    double total_path_length_normalized = 0;
    double total_path_shorten_time_normalized = 0;
    double total_path_length_shortened_normalized = 0;
    for (ObjectPairExperimentResult object_pair_stat : object_pair_stats) {
        if (object_pair_stat.success) {
            success_count++;
            total_path_find_time += object_pair_stat.path_find_time;
            total_path_length += object_pair_stat.path_length;
            total_path_shorten_time += object_pair_stat.path_shorten_time;
            total_path_length_shortened += object_pair_stat.path_length_shortened;

            total_path_find_time_normalized += object_pair_stat.path_find_time / object_pair_stat.euclidean_distance;
            total_path_length_normalized += object_pair_stat.path_length / object_pair_stat.euclidean_distance;
            total_path_shorten_time_normalized += object_pair_stat.path_shorten_time / object_pair_stat.euclidean_distance;
            total_path_length_shortened_normalized += object_pair_stat.path_length_shortened / object_pair_stat.euclidean_distance;
        }
    }
    double succuss_rate = (double)success_count / object_pair_stats.size();
    double mean_path_find_time = total_path_find_time / success_count;
    double mean_path_length = total_path_length / success_count;
    double mean_path_shorten_time = total_path_shorten_time / success_count;
    double mean_path_length_shortened = total_path_length_shortened / success_count;
    double mean_path_find_time_normalized = total_path_find_time_normalized / success_count;
    double mean_path_length_normalized = total_path_length_normalized / success_count;
    double mean_path_shorten_time_normalized = total_path_shorten_time_normalized / success_count;
    double mean_path_length_shortened_normalized = total_path_length_shortened_normalized / success_count;

    cout << "Number of nodes: " << result.num_nodes << endl;

    result.n_experiments = object_pair_stats.size();
    result.n_successes = success_count;
    result.success_rate = succuss_rate;
    result.mean_path_find_time = mean_path_find_time;
    result.mean_path_length = mean_path_length;
    result.mean_path_shorten_time = mean_path_shorten_time;
    result.mean_path_length_shortened = mean_path_length_shortened;
    result.mean_path_find_time_normalized = mean_path_find_time_normalized;
    result.mean_path_length_normalized = mean_path_length_normalized;
    result.mean_path_shorten_time_normalized = mean_path_shorten_time_normalized;
    result.mean_path_length_shortened_normalized = mean_path_length_shortened_normalized;

    cout << "Number of nodes: " << result.num_nodes << endl;
    cout << "Number of edges: " << result.num_edges << endl;
    cout << "Number of components: " << result.num_components << endl;
    cout << "Mean number of neighbors: " << result.mean_num_neighbors << endl;
    cout << "Success rate: " << result.success_rate << endl;
    cout << "Mean path find time: " << result.mean_path_find_time << endl;
    cout << "Mean path length: " << result.mean_path_length << endl;
    cout << "Mean path shorten time: " << result.mean_path_shorten_time << endl;
    cout << "Mean path length shortened: " << result.mean_path_length_shortened << endl;

    string output_filename = generateFilename();
    auto config_json = generateConfigJson(skeletonfinder);
    saveStats(result, config_json, output_filename);

    string experiment_name = _experiment_name.substr(0, _experiment_name.find_first_of(" "));
    // save valid nodes to file
    skeletonfinder.saveNodes(_save_path + "/nodes_" + "" +  experiment_name + ".pcd");

    // save connections to file
    skeletonfinder.saveConnections(_save_path + "/connections_" + "" +  experiment_name + ".txt");

    // save adjacency matrix to file
    skeletonfinder.saveAdjMatrix(_save_path + "/adjacency_matrix_" + "" +  experiment_name + ".txt");

    return result;
}