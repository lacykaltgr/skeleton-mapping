#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <torch/torch.h>
#include <c10/cuda/CUDAStream.h>  // For CUDA streams

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <filesystem>

using json = nlohmann::json;
namespace fs = std::filesystem;

struct Camera {
    int width;
    int height;
    double fx, fy, cx, cy;
    double depth_scale;
};

std::string format_int_to_6chars(int i) {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << i;
    return oss.str();
}

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

torch::Tensor readPoseFile(const std::string& path) {
    std::ifstream file(path);
    json j;
    file >> j;
    j = j["transform"];

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = Eigen::Quaternionf(j["rotation"]["w"], j["rotation"]["x"], j["rotation"]["y"], j["rotation"]["z"]).toRotationMatrix();
    pose.block<3, 1>(0, 3) << j["translation"]["x"], j["translation"]["y"], j["translation"]["z"];
    torch::Tensor pose_tensor = torch::from_blob(pose.data(), {4, 4}, torch::dtype(torch::kFloat32));
    return pose_tensor;
}

int main(int argc, char **argv) {
    try {
        std::string input_directory = argv[1];
        std::string output_path = argv[2];

        float min_z = 0.1f, max_z = 30.0f;
        int pixel_step_size = 4, median_filter_size = 0;
        int image_step_size = 10;

        Camera camera = readCameraInfo(input_directory + "/udist_camera_matrix.json");

        std::string depth_folder = input_directory + "/depth";
        std::string color_folder = input_directory + "/color";
        std::string poses_folder = input_directory + "/tf";

        size_t num_files = 0;
        for (const auto& entry : fs::directory_iterator(poses_folder)) {
            if (entry.path().extension() == ".json" && entry.path().filename() != "camera_info.json") {
                num_files++;
            }
        }
        if (num_files == 0) {
            std::cerr << "No poses loaded." << std::endl;
            return -1;
        }

        std::cout << "Creating point cloud ..." << std::endl;
        PointCloud pointCloud;

        torch::Device device(torch::kCUDA);

        // Create CUDA stream for asynchronous processing
        at::cuda::CUDAStream stream = at::cuda::getStreamFromPool();

        for (size_t i = 0; i < num_files; i += image_step_size) {
            std::stringstream depth_image_path, color_image_path;
            depth_image_path << depth_folder << "/" << format_int_to_6chars(i) << ".png";
            color_image_path << color_folder << "/" << format_int_to_6chars(i) << ".png";

            cv::Mat depth_image = cv::imread(depth_image_path.str(), cv::IMREAD_UNCHANGED);
            depth_image.convertTo(depth_image, CV_32F, 1.0f / camera.depth_scale);
            cv::Mat color_image = cv::imread(color_image_path.str(), cv::IMREAD_COLOR);
            if (depth_image.empty() || color_image.empty()) {
                std::cerr << "Could not open depth or color image for index " << i << std::endl;
                continue;
            }

            std::string pose_path = poses_folder + "/" + format_int_to_6chars(i) + "_tf.json";
            auto pose = readPoseFile(pose_path);

            if (median_filter_size > 0) {
                cv::medianBlur(depth_image, depth_image, median_filter_size);
            }

            // Convert OpenCV Mat to Torch tensor and move to GPU
            torch::Tensor depth_tensor = torch::from_blob(depth_image.data, {depth_image.rows, depth_image.cols}, torch::kFloat32).to(device) / camera.depth_scale;
            torch::Tensor color_tensor = torch::from_blob(color_image.data, {color_image.rows, color_image.cols, 3}, torch::kByte).to(device);

            // Mask for depth in valid range
            torch::Tensor valid_mask = (depth_tensor >= min_z) & (depth_tensor <= max_z);

            // Generate pixel grid (u, v)
            std::vector<torch::Tensor> grid = torch::meshgrid({
                torch::arange(0, depth_image.rows, pixel_step_size, device), 
                torch::arange(0, depth_image.cols, pixel_step_size, device)}
            );
            torch::Tensor v = grid[0].contiguous();
            torch::Tensor u = grid[1].contiguous();
            torch::Tensor depth_filtered = depth_tensor.index({v, u}).masked_select(valid_mask);

            // Calculate x, y, z
            torch::Tensor x = (u - camera.cx) * depth_filtered / camera.fx;
            torch::Tensor y = (v - camera.cy) * depth_filtered / camera.fy;
            torch::Tensor z = depth_filtered;

            // Stack the (x, y, z, 1) points
            torch::Tensor points_camera = torch::stack({x, y, z}, 1);

            // Apply pose transformation (pose.inverse())
            torch::Tensor rotation_matrix = pose.slice(0, 0, 2).slice(1, 0, 2);
            torch::Tensor translation_vector = pose.slice(0, 0, 2).slice(1, 2, 3);

            torch::Tensor points_world = rotation_matrix * points_camera + translation_vector;

            // Extract color and add to point cloud
            std::vector<torch::Tensor> bgr_image  = color_tensor.split(1, -1);  
            torch::Tensor b = bgr_image[0].contiguous();
            torch::Tensor g = bgr_image[1].contiguous();
            torch::Tensor r = bgr_image[2].contiguous();
            
            for (int j = 0; j < points_world.size(0); ++j) {
                PointType point;
                point.x = points_world[j][0].item<float>();
                point.y = points_world[j][1].item<float>();
                point.z = points_world[j][2].item<float>();

                point.r = r[j].item<uint8_t>();
                point.g = g[j].item<uint8_t>();
                point.b = b[j].item<uint8_t>();

                pointCloud.push_back(point);
            }
        }

        std::cout << "Saving point cloud as PCD..." << std::endl;
        pcl::io::savePCDFileBinaryCompressed(output_path, pointCloud);

    } catch (std::exception& e) {
        std::cout << "Exception occurred: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
