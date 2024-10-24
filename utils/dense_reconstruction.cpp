#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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

Eigen::Matrix4f readPoseFile(const std::string& path) {
    std::ifstream file(path);
    json j;
    file >> j;
    j = j["transform"];

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3, 3>(0, 0) = Eigen::Quaternionf(j["rotation"]["w"], j["rotation"]["x"], j["rotation"]["y"], j["rotation"]["z"]).toRotationMatrix();
    pose.block<3, 1>(0, 3) << j["translation"]["x"], j["translation"]["y"], j["translation"]["z"];
    return pose;
}
    

int main(int argc, char **argv)
{
    try {

        std::string input_directory = argv[1];
        std::string output_path = argv[2];  

        float min_z = 0.2f, max_z = 20.0f;
        int pixel_step_size = 4, median_filter_size = 0;
        int image_step_size = 5;

        if (argc > 3) {
            pixel_step_size = std::stoi(argv[3]);
        }
        if (argc > 4) {
            min_z = std::stof(argv[4]);
        }

        Camera camera = readCameraInfo(input_directory + "/udist_camera_matrix.json");

        std::string depth_folder = input_directory + "/depth";
        std::string color_folder = input_directory + "/color";
        std::string poses_folder = input_directory + "/tf";    

        // Load poses
        //std::vector<Eigen::Matrix4f> poses = loadPoses(poses_file);
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

        std::cout << "Output path: " << output_path << std::endl;

        std::cout << "Creating point cloud ..." << std::endl;
        PointCloud pointCloud;

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

            for (int v = 0; v < depth_image.rows; v += pixel_step_size) {
                for (int u = 0; u < depth_image.cols; u += pixel_step_size) {
                    float depth = depth_image.at<float>(v, u);

                    if (depth < min_z || depth > max_z) {
                        continue;
                    }

                    float x = (u - camera.cx) * depth / camera.fx;
                    float y = (v - camera.cy) * depth / camera.fy;
                    float z = depth;

                    Eigen::Vector3f point_world = pose.block<3, 3>(0, 0) * Eigen::Vector3f(x, y, z) + pose.block<3, 1>(0, 3);

                    PointType point;
                    point.x = point_world(0);
                    point.y = point_world(1);
                    point.z = point_world(2);

                    if (color_image.channels() == 3) {
                        point.r = color_image.at<cv::Vec3b>(v, u)[2];
                        point.g = color_image.at<cv::Vec3b>(v, u)[1];
                        point.b = color_image.at<cv::Vec3b>(v, u)[0];
                    } else {
                        uchar gray_value = color_image.at<uchar>(v, u);
                        point.r = gray_value;
                        point.g = gray_value;
                        point.b = gray_value;
                    }

                    pointCloud.push_back(point);
                }
            }
        }

        
        std::cout << "The new point cloud contains " << pointCloud.size() << " points." << std::endl;
        std::cout << "Saving point cloud ..." << std::endl;
        std::cout << "Saving point cloud as PCD..." << std::endl;
        pcl::io::savePCDFileBinaryCompressed(output_path, pointCloud);

    }
    catch (std::exception& e) {
        std::cout << "Exception occured: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
