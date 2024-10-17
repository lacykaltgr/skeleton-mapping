#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <string>

void loadAndCutColoredPointCloud(const std::string& pcdFilePath, float zThreshold, const std::string& outputFilePath) {
    // Create a PointCloud object to hold the input data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load the point cloud from the .pcd file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdFilePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return;
    }
    std::cout << "Loaded colored point cloud from " << pcdFilePath << " with " << cloud->size() << " points.\n";

    // Create a PassThrough filter to filter out points based on the Z threshold
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<float>::max(), zThreshold);

    // Create a PointCloud to hold the filtered output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*filteredCloud);

    std::cout << "Filtered point cloud has " << filteredCloud->size() << " points.\n";

    // Save the filtered point cloud to a new .pcd file
    if (pcl::io::savePCDFileBinary(outputFilePath, *filteredCloud) == -1) {
        PCL_ERROR("Couldn't write the output PCD file\n");
        return;
    }
    std::cout << "Filtered colored point cloud saved to " << outputFilePath << "\n";
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <z_threshold> <output_pcd_file>\n";
        return -1;
    }

    std::string pcdFilePath = argv[1];
    float zThreshold = std::stof(argv[2]);
    std::string outputFilePath = argv[3];

    loadAndCutColoredPointCloud(pcdFilePath, zThreshold, outputFilePath);

    return 0;
}
