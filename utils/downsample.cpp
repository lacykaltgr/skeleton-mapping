#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    // Check if the proper number of arguments is provided
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.pcd> <leaf_size>" << std::endl;
        return -1;
    }

    // Parse input arguments
    std::string input_file = argv[1];
    std::string output_file = argv[2];
    double leaf_size = std::stod(argv[3]);

    // Load the input PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
        std::cerr << "Error: Couldn't read input PCD file " << input_file << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_file << std::endl;

    // Downsample the point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);

    // Save the downsampled point cloud to output file
    pcl::io::savePCDFileASCII(output_file, *cloud_filtered);
    std::cout << "Saved " << cloud_filtered->width * cloud_filtered->height << " data points to " << output_file << std::endl;

    return 0;
}