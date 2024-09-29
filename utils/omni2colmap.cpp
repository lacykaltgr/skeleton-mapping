#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
#if defined(__APPLE__) && __cplusplus < 201703L
namespace fs = std::__fs::filesystem;
#else
namespace fs = std::filesystem;
#endif

struct Camera {
    int width;
    int height;
    double fx, fy, cx, cy;
    double depth_scale;
};

struct Pose {
    int image_id;
    std::string image_name;
    double qw, qx, qy, qz;  // Quaternion
    double tx, ty, tz;      // Translation
};

Camera readCameraInfo(const std::string& filename) {
    std::ifstream file(filename);
    json j;
    file >> j;

    Camera camera;
    camera.width = j["width"];
    camera.height = j["height"];
    camera.fx = j["camera_matrix"][0][0];
    camera.fy = j["camera_matrix"][1][1];
    camera.cx = j["camera_matrix"][0][2];
    camera.cy = j["camera_matrix"][1][2];
    camera.depth_scale = j["depth_scale"];

    return camera;
}

std::vector<Pose> readJsonFiles(const std::string& directory) {
    std::vector<Pose> poses;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".json" && entry.path().filename() != "camera_info.json") {
            std::ifstream file(entry.path());
            json j;
            file >> j;

            Pose pose;
            pose.image_id = j["image_id"];
            pose.image_name = j["image_name"];
            pose.qw = j["rotation"][0];
            pose.qx = j["rotation"][1];
            pose.qy = j["rotation"][2];
            pose.qz = j["rotation"][3];
            pose.tx = j["translation"][0];
            pose.ty = j["translation"][1];
            pose.tz = j["translation"][2];

            poses.push_back(pose);
        }
    }
    return poses;
}

void writeColmapCameras(const Camera& camera, const std::vector<Pose>& poses, const std::string& output_file) {
    std::ofstream out(output_file);
    out << "# Camera list with one line of data per camera:\n";
    out << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
    out << "# Number of cameras: 1\n";

    // Write camera information
    out << "1 PINHOLE " << camera.width << " " << camera.height << " "
        << camera.fx << " " << camera.fy << " " << camera.cx << " " << camera.cy << "\n";

    out << "\n# Image list with two lines of data per image:\n";
    out << "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n";
    out << "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";
    out << "# Number of images: " << poses.size() << ", mean observations per image: 0\n";

    for (const auto& pose : poses) {
        out << pose.image_id << " " << pose.qw << " " << pose.qx << " " << pose.qy << " " << pose.qz
            << " " << pose.tx << " " << pose.ty << " " << pose.tz << " 1 " << pose.image_name << "\n\n";
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_directory> <output_file>\n";
        return 1;
    }

    std::string input_directory = argv[1];
    std::string output_file = argv[2];

    Camera camera = readCameraInfo(input_directory + "/udist_camera_matrix.json");
    std::vector<Pose> poses = readJsonFiles(input_directory);
    writeColmapCameras(camera, poses, output_file);

    std::cout << "Conversion complete. Output written to " << output_file << std::endl;
    return 0;
}