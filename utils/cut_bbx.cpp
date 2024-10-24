#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <string>

void removePointsWithinBoundingBox(const std::string& input_pcd, const std::string& output_pcd,
                                   float xmin, float ymin, float zmin,
                                   float xmax, float ymax, float zmax) {
    // Load the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_pcd, *cloud) == -1) {
        PCL_ERROR("Couldn't read the input PCD file\n");
        return;
    }

    int og_size = cloud->size();
    std::cout << "Loaded point cloud from " << input_pcd << " with " << og_size << " points." << std::endl;

    // Define the bounding box
    Eigen::Vector4f min_point(xmin, ymin, zmin, 1.0);
    Eigen::Vector4f max_point(xmax, ymax, zmax, 1.0);

    // CropBox filter to remove points inside the bounding box
    pcl::CropBox<pcl::PointXYZRGB> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setMin(min_point);
    crop_box.setMax(max_point);
    crop_box.setNegative(true); // Set to true to keep points outside the box

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    crop_box.filter(*cloud_filtered);

    std::cout << "Removed " << og_size - cloud_filtered->size() << " points within the bounding box." << std::endl;

    // Save the filtered point cloud
    pcl::io::savePCDFileBinary(output_pcd, *cloud_filtered);
    std::cout << "Filtered point cloud saved to " << output_pcd << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 9) {
        std::cerr << "Usage: " << argv[0] << " input.pcd output.pcd xmin ymin zmin xmax ymax zmax" << std::endl;
        return -1;
    }

    // Parse command line arguments
    std::string input_pcd = argv[1];
    std::string output_pcd = argv[2];
    float xmin = std::stof(argv[3]);
    float xmax = std::stof(argv[4]);
    float ymin = std::stof(argv[5]);
    float ymax = std::stof(argv[6]);
    float zmin = std::stof(argv[7]);
    float zmax = std::stof(argv[8]);

    std::cout << "Cutting point cloud " << input_pcd << " with bounding box: "
         << "xmin=" << xmin << ", ymin=" << ymin << ", zmin=" << zmin << ", "
         << "xmax=" << xmax << ", ymax=" << ymax << ", zmax=" << zmax << std::endl;

    // Call the function to remove points within the bounding box
    removePointsWithinBoundingBox(input_pcd, output_pcd, xmin, ymin, zmin, xmax, ymax, zmax);

    return 0;
}
