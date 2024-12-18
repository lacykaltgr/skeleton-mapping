#ifndef EXPERIMENTS_H
#define EXPERIMENTS_H

#include "SkeletonFinder/skeleton_finder_3D.h"
#include "SkeletonFinder/objects.h"

struct ObjectPairExperimentResult {
    string object_id1;
    string object_id2;
    bool success;
    double path_length;
    double path_length_shortened;
    double path_find_time;
    double path_shorten_time;
    double euclidean_distance;
};


struct ExperimentResult {
    int num_nodes;
    int num_edges;
    int num_components;
    int mean_num_neighbors;
    int n_experiments;
    int n_successes;    
    double success_rate;
    double mean_path_find_time;
    double mean_path_length;
    double mean_path_shorten_time;
    double mean_path_length_shortened;
    double mean_path_find_time_normalized;
    double mean_path_length_normalized;
    double mean_path_shorten_time_normalized;
    double mean_path_length_shortened_normalized;
    vector<ObjectPairExperimentResult> object_pair_stats;
};

inline nlohmann::json generateConfigJson(SkeletonFinder skeletonfinder) {
    nlohmann::json j;
    std::map<std::string, double> config_map = skeletonfinder.getConfigMap();
    for (auto& it : config_map) {
        j[it.first] = it.second;
    }
    return j;
}

class SkeletonFinderExperiment {

    private:
        int getNumNodes(SkeletonFinder skeletonfinder);
        int getNumEdges(SkeletonFinder skeletonfinder);
        int getNumComponents(SkeletonFinder skeletonfinder);
        vector<ObjectPairExperimentResult> getObjectPairStats(SkeletonFinder skeletonfinder, vector<MapObject>& objects);

        string _experiment_name;
        string _scene_name;
        string _save_path;
        string _dimension;
        

        // date and time
        time_t _timestamp;
        
        inline string generateFilename() {
            string experiment_name = _experiment_name.substr(0, _experiment_name.find_first_of(" "));
            return _save_path + "/experiment_" + experiment_name + "_" + _dimension + "_" + _scene_name + ".json";
        }


        inline void printMessage() {
            cout << "Running Experiment" << endl;
            cout << "------------------" << endl;
            cout << "Experiment name: " << _experiment_name << endl;
            cout << "Scene name: " << _scene_name << endl;
            cout << "Dimension: " << _dimension << endl;
            cout << "Save path: " << _save_path << endl;
        }

        inline void show_progress_bar(float progress) {
            int bar_width = 50;
            std::cout << "[";
            int pos = bar_width * progress;
            for (int i = 0; i < bar_width; ++i) {
                if (i < pos) {
                    std::cout << "=";
                } else if (i == pos) {
                    std::cout << ">";
                } else {
                    std::cout << " ";
                }
            }
            std::cout << "] " << int(progress * 100.0) << " %\r";
            std::cout.flush();
        }


    public:

        SkeletonFinderExperiment(const string& experiment_name, int robot_type, const string& input_path) {
            _timestamp = time(0);
            _experiment_name = experiment_name;
            _scene_name = input_path.substr(input_path.find_last_of("/\\") + 1);
            _save_path = input_path.substr(0, input_path.find_last_of("/\\"));

            if (robot_type == 0) {
                _dimension = "2D";
            } else {
                _dimension = "3D";
            }

            printMessage();
        }



        inline void saveStats(
            ExperimentResult& result, nlohmann::json config,  const string& filename
        ) {
            ofstream file(filename);
            nlohmann::json j;
            j["timestamp"] = _timestamp;
            j["experiment_name"] = _experiment_name;
            j["scene_name"] = _scene_name;
            j["dimension"] = _dimension;
            j["num_nodes"] = result.num_nodes;
            j["num_edges"] = result.num_edges;
            j["mean_num_neighbors"] = result.mean_num_neighbors;
            j["success_rate"] = result.success_rate;
            j["mean_path_find_time"] = result.mean_path_find_time;
            j["mean_path_length"] = result.mean_path_length;
            j["mean_path_shorten_time"] = result.mean_path_shorten_time;
            j["mean_path_length_shortened"] = result.mean_path_length_shortened;
            j["mean_path_find_time_normalized"] = result.mean_path_find_time_normalized;
            j["mean_path_length_normalized"] = result.mean_path_length_normalized;
            j["mean_path_shorten_time_normalized"] = result.mean_path_shorten_time_normalized;
            j["mean_path_length_shortened_normalized"] = result.mean_path_length_shortened_normalized;


            nlohmann::json object_pair_stats;
            for (ObjectPairExperimentResult& object_pair_stat : result.object_pair_stats) {
                nlohmann::json object_pair_stat_json;
                object_pair_stat_json["object_id1"] = object_pair_stat.object_id1;
                object_pair_stat_json["object_id2"] = object_pair_stat.object_id2;
                object_pair_stat_json["success"] = object_pair_stat.success;
                object_pair_stat_json["path_length"] = object_pair_stat.path_length;
                object_pair_stat_json["path_length_shortened"] = object_pair_stat.path_length_shortened;
                object_pair_stat_json["path_find_time"] = object_pair_stat.path_find_time;
                object_pair_stat_json["path_shorten_time"] = object_pair_stat.path_shorten_time;
                object_pair_stat_json["euclidean_distance"] = object_pair_stat.euclidean_distance;
                object_pair_stats.push_back(object_pair_stat_json);
            }
            j["z_config"] = config;
            j["z_object_pair_stats"] = object_pair_stats;
            

            file << j.dump(4);
            cout << "Stats saved to file: " << filename << endl;
        }

        ExperimentResult run_experiments(SkeletonFinder skeletonfinder, const string& obj_filename);

};

#endif