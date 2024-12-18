#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <string>

void loadAndFlipPointCloud(const std::string& pcdFilePath, const std::string& outputFilePath) {
    // Create a PointCloud object to hold the input data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Load the point cloud from the .pcd file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdFilePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return;
    }
    std::cout << "Loaded colored point cloud from " << pcdFilePath << " with " << cloud->size() << " points.\n";

    // Create a PointCloud to hold the flipped output
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr flippedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    flippedCloud->resize(cloud->size());

    // Flip the points along the Z dimension
    for (size_t i = 0; i < cloud->size(); ++i) {
        flippedCloud->points[i].x = cloud->points[i].x;     // x remains the same
        flippedCloud->points[i].y = cloud->points[i].y;     // y remains the same
        flippedCloud->points[i].z = -cloud->points[i].z;    // z is flipped
        flippedCloud->points[i].rgba = cloud->points[i].rgba; // preserve color information
    }

    std::cout << "Flipped point cloud has " << flippedCloud->size() << " points.\n";

    // Save the flipped point cloud to a new .pcd file
    if (pcl::io::savePCDFileBinary(outputFilePath, *flippedCloud) == -1) {
        PCL_ERROR("Couldn't write the output PCD file\n");
        return;
    }
    std::cout << "Flipped colored point cloud saved to " << outputFilePath << "\n";
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file> <output_pcd_file>\n";
        return -1;
    }

    std::string pcdFilePath = argv[1];
    std::string outputFilePath = argv[2];

    loadAndFlipPointCloud(pcdFilePath, outputFilePath);

    return 0;
}
