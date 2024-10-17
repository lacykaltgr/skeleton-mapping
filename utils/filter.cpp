// File: point_cloud_denoising.cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/median_filter.h>
//#include <pcl/filters/bilateral.h>
//#include <pcl/filters/shadowpoints.h>

using PointT = pcl::PointXYZRGB;

void printPointReduction(const std::string& filter_name, int before, int after) {
    std::cout << filter_name << ": " << (before - after) << " points removed (" 
              << before << " -> " << after << ")" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd> <output_pcd>" << std::endl;
        return -1;
    }

    // Load input PCD file
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud_filtered) == -1) {
        PCL_ERROR("Couldn't read the input PCD file\n");
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud_filtered->size() << " points." << std::endl;

    int original_size = cloud_filtered->size();
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);


    /*
    // Step 1: Shadow Points Removal
    pcl::ShadowPoints<PointT, PointT> shadow_filter;
    shadow_filter.setInputCloud(cloud);
    shadow_filter.setThreshold(0.1); // Adjust threshold as needed
    shadow_filter.filter(*cloud_filtered);
    printPointReduction("ShadowPoints Removal", original_size, cloud_filtered->size());
    original_size = cloud_filtered->size();
    */

    // Step 2: Statistical Outlier Removal
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);
    printPointReduction("StatisticalOutlierRemoval", original_size, cloud->size());
    original_size = cloud->size();

    // Step 3: Radius Outlier Removal
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(2);
    ror.filter(*cloud_filtered);
    printPointReduction("RadiusOutlierRemoval", original_size, cloud_filtered->size());
    original_size = cloud_filtered->size();

    // Step 4: Median Filter
    pcl::MedianFilter<PointT> median_filter;
    median_filter.setInputCloud(cloud_filtered);
    median_filter.setWindowSize(5); // Adjust window size as needed
    median_filter.applyFilter(*cloud);
    printPointReduction("MedianFilter", original_size, cloud->size());
    //original_size = cloud->size();

    /*
    // Step 5: Bilateral Filter
    pcl::BilateralFilter<PointT> bilateral_filter;
    bilateral_filter.setInputCloud(cloud);
    bilateral_filter.setHalfSize(0.5);  // Spatial sigma
    bilateral_filter.setStdDev(0.05);   // Range sigma
    bilateral_filter.applyFilter(*cloud_filtered);
    printPointReduction("BilateralFilter", original_size, cloud_filtered->size());
    //original_size = cloud_filtered->size();
    */

    // Save the filtered cloud to output PCD file
    if (pcl::io::savePCDFileBinary(argv[2], *cloud) == -1) {
        PCL_ERROR("Couldn't write the output PCD file\n");
        return -1;
    }

    std::cout << "Filtered point cloud saved to " << argv[2] << " with " 
              << cloud->size() << " points." << std::endl;

    return 0;
}
