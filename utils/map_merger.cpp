#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " output_file.pcd input1.pcd input2.pcd ..." << std::endl;
        return -1;
    }

    // Create a point cloud to hold the merged result with XYZ and RGB attributes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Loop over all input files
    for (int i = 2; i < argc; ++i) {
        std::string input_file = argv[i];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Load the PCD file with XYZRGB points
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(input_file, *cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", input_file.c_str());
            return -1;
        }

        std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << input_file << std::endl;

        // Merge the current cloud into the merged cloud
        *merged_cloud += *cloud;
    }

    // Save the merged cloud into the output file
    std::string output_file = argv[1];
    pcl::io::savePCDFileASCII(output_file, *merged_cloud);
    std::cout << "Merged point cloud saved as " << output_file << " with " 
              << merged_cloud->width * merged_cloud->height << " points." << std::endl;

    return 0;
}
