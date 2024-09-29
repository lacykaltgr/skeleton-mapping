// Test.cpp for testing PointCloud.h Page class implementing RTree spatial index

#define _CRT_SECURE_NO_WARNINGS

#include "C-Tree-P/Provider.h"
#include <pcl/visualization/cloud_viewer.h>  // PCL visualization header
#include <pcl/kdtree/kdtree_flann.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <ctime>

double random(double x1, double x2)
{
    return x1 + rand() * (x2 - x1) / RAND_MAX;
}

Provider provider;  // Point cloud provider

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Create a PCL Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();

    // Keep the visualizer window open until the user closes it
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char *argv[])
{
    // Seed the random number generator using the current time
    std::srand(static_cast<unsigned int>(std::time(0)));

    Point pt;
    vector<Point> points;

    // Default PCD file path
    const char *default_pcd_path = "/Users/czimbermark/Documents/TDK/data/sac_plane_test.pcd";

    // Check if a command-line argument is provided
    const char *file_path;
    if (argc < 2) {
        printf("No PCD file provided, using default: %s\n", default_pcd_path);
        file_path = default_pcd_path;
    } else {
        file_path = argv[1];  // Use the provided PCD file path
        printf("Loading point cloud from: %s\n", file_path);
    }

    // Load point cloud and measure indexing time
    double index_time = provider.ReadPCD(file_path);

    // Get bounding box of the point cloud
    Bound bound;
    provider.GetBound(bound);
    printf("Cloud Boundaries: X[%f, %f], Y[%f, %f], Z[%f, %f]\n",
           bound.x1, bound.x2, bound.y1, bound.y2, bound.z1, bound.z2);

    // Test Nearest Neighbor Search
    printf("Testing nearest neighbor search...\n");

    // Generate a random query point within the cloud bounds
    Point queryPoint;
    queryPoint.x = random(bound.x1, bound.x2);
    queryPoint.y = random(bound.y1, bound.y2);
    queryPoint.z = random(bound.z1, bound.z2);

    printf("Query Point: (%.2f, %.2f, %.2f)\n", queryPoint.x, queryPoint.y, queryPoint.z);
    std::vector<double> distance_ctp(1);

    clock_t start = clock();  // Start measuring time for nearest neighbor search
    Point nearestPoint = provider.NearestNeighbourQuery(queryPoint, distance_ctp, 100.0); // Assuming 100.0 as the max distance
    clock_t end = clock();    // End measuring time

    double provider_time = ((double)(end - start)) / CLOCKS_PER_SEC;
    printf("Nearest neighbor found in %f seconds\n", provider_time);
    printf("Nearest Point: (%.2f, %.2f, %.2f)\n", nearestPoint.x, nearestPoint.y, nearestPoint.z);
    printf("Nearest neighbor found at distance: %f\n", distance_ctp[0]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = provider.GetPointCloud();  // Get the loaded point cloud

    // Build a PCL k-d tree
    printf("Building PCL k-d tree...\n");
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    
    clock_t start_kdtree_build = clock();  // Start measuring time for k-d tree build
    kdtree.setInputCloud(cloud);
    clock_t end_kdtree_build = clock();  // End measuring time for k-d tree build
    
    double kdtree_build_time = ((double)(end_kdtree_build - start_kdtree_build)) / CLOCKS_PER_SEC;
    printf("PCL k-d tree built in %f seconds\n", kdtree_build_time);

    // Perform nearest neighbor search using the PCL k-d tree
    printf("Testing nearest neighbor search with PCL k-d tree...\n");

    pcl::PointXYZ searchPoint;
    searchPoint.x = queryPoint.x;
    searchPoint.y = queryPoint.y;
    searchPoint.z = queryPoint.z;

    std::vector<int> nearest(1);
    std::vector<float> distance_kd(1);

    clock_t start_kdtree_search = clock();  // Start measuring time for k-d tree search
    if (kdtree.nearestKSearch(searchPoint, 1, nearest, distance_kd) > 0)
    {
        clock_t end_kdtree_search = clock();  // End measuring time for k-d tree search
        double kdtree_search_time = ((double)(end_kdtree_search - start_kdtree_search)) / CLOCKS_PER_SEC;

        printf("PCL k-d tree nearest neighbor found in %f seconds\n", kdtree_search_time);
        pcl::PointXYZ nearestPCLPoint = cloud->points[nearest[0]];
        printf("Nearest Point (PCL): (%.2f, %.2f, %.2f)\n", nearestPCLPoint.x, nearestPCLPoint.y, nearestPCLPoint.z);
        printf("Nearest neighbor found at distance: %f\n", distance_kd[0]);

        // Calculate the ratio of Provider search time vs. k-d tree search time
        if (kdtree_search_time > 0)  // Avoid division by zero
        {
            double time_ratio = kdtree_search_time / provider_time;
            double index_ratio = kdtree_build_time / index_time;
            printf("C-Tree-P nearest neighbor search is %f times faster than K-d tree search.\n", time_ratio);
            printf("C-Tree-P tree build is %f times faster than K-d tree build.\n", index_ratio);
        }
    }

    // Visualize the loaded point cloud
    // printf("Visualizing point cloud...\n");
    // visualizePointCloud(cloud);  // Visualize it

    return 0;
}