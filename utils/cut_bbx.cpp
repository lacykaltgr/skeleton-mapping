#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
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

    // Create a new cloud for filtered points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    // Reserve space for efficiency (assuming most points will be kept)
    cloud_filtered->reserve(cloud->size());

    // Manually check each point
    for (const auto& point : cloud->points) {
        // Check if point is outside the bounding box
        if (point.x < xmin || point.x > xmax ||
            point.y < ymin || point.y > ymax ||
            point.z < zmin || point.z > zmax) {
            // Point is outside the box, keep it
            cloud_filtered->push_back(point);
        }
    }

    // Resize to actual number of points (in case reserve allocated too much)
    cloud_filtered->resize(cloud_filtered->points.size());
    
    std::cout << "Removed " << (og_size - cloud_filtered->size()) << " points within the bounding box." << std::endl;
    std::cout << "Remaining points: " << cloud_filtered->size() << std::endl;

    // Save the filtered point cloud
    if (pcl::io::savePCDFileBinary(output_pcd, *cloud_filtered) == -1) {
        PCL_ERROR("Couldn't save the filtered PCD file\n");
        return;
    }
    std::cout << "Filtered point cloud saved to " << output_pcd << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 9) {
        std::cerr << "Usage: " << argv[0] << " input.pcd output.pcd xmin ymin zmin xmax ymax zmax" << std::endl;
        return -1;
    }

    try {
        // Parse command line arguments
        std::string input_pcd = argv[1];
        std::string output_pcd = argv[2];
        float xmin = std::stof(argv[3]);
        float ymin = std::stof(argv[4]);
        float zmin = std::stof(argv[5]);
        float xmax = std::stof(argv[6]);
        float ymax = std::stof(argv[7]);
        float zmax = std::stof(argv[8]);

        // Validate bounds
        if (xmax < xmin || ymax < ymin || zmax < zmin) {
            std::cerr << "Error: Max values must be greater than min values" << std::endl;
            return -1;
        }

        std::cout << "Cutting point cloud " << input_pcd << " with bounding box: "
             << "xmin=" << xmin << ", ymin=" << ymin << ", zmin=" << zmin << ", "
             << "xmax=" << xmax << ", ymax=" << ymax << ", zmax=" << zmax << std::endl;

        removePointsWithinBoundingBox(input_pcd, output_pcd, xmin, ymin, zmin, xmax, ymax, zmax);
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}