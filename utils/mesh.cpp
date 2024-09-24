#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Poisson_surface_reconstruction_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/PLY_writer.h>
#include <tiny_gltf.h> // Include the tinygltf header
#include <vector>
#include <fstream>

// Define types for CGAL kernel and mesh
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point_3;
typedef Kernel::Vector_3 Vector_3;
typedef std::pair<Point_3, Vector_3> PointVectorPair;
typedef CGAL::Surface_mesh<Point_3> Mesh;

// Function to save the mesh in GLTF format
bool saveMeshAsGLTF(const Mesh& mesh, const std::string& filename) {
    tinygltf::Model model;
    tinygltf::Scene scene;
    model.defaultScene = 0;
    model.scenes.push_back(scene);

    // Prepare buffers
    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<unsigned int> indices;

    for (const auto& vertex : mesh.vertices()) {
        const Point_3& p = mesh.point(vertex);
        positions.push_back(static_cast<float>(p.x()));
        positions.push_back(static_cast<float>(p.y()));
        positions.push_back(static_cast<float>(p.z()));

        if (mesh.has_normals()) {
            const Vector_3& n = mesh.normal(vertex);
            normals.push_back(static_cast<float>(n.x()));
            normals.push_back(static_cast<float>(n.y()));
            normals.push_back(static_cast<float>(n.z()));
        }
    }

    for (const auto& face : mesh.faces()) {
        for (const auto& halfedge : CGAL::halfedges_around_face(mesh.halfedge(face), mesh)) {
            indices.push_back(static_cast<unsigned int>(mesh.target(halfedge)));
        }
    }

    // Fill buffers into tinygltf model
    // Buffer for positions
    tinygltf::Buffer position_buffer;
    position_buffer.data.assign(reinterpret_cast<const unsigned char*>(positions.data()),
                                reinterpret_cast<const unsigned char*>(positions.data() + positions.size() * sizeof(float)));
    model.buffers.push_back(position_buffer);

    // Buffer for normals
    if (!normals.empty()) {
        tinygltf::Buffer normal_buffer;
        normal_buffer.data.assign(reinterpret_cast<const unsigned char*>(normals.data()),
                                  reinterpret_cast<const unsigned char*>(normals.data() + normals.size() * sizeof(float)));
        model.buffers.push_back(normal_buffer);
    }

    // Buffer for indices
    tinygltf::Buffer index_buffer;
    index_buffer.data.assign(reinterpret_cast<const unsigned char*>(indices.data()),
                             reinterpret_cast<const unsigned char*>(indices.data() + indices.size() * sizeof(unsigned int)));
    model.buffers.push_back(index_buffer);

    // Add buffers to the model
    model.bufferViews.push_back({}); // Position buffer view
    model.bufferViews.push_back({}); // Normal buffer view (if available)
    model.bufferViews.push_back({}); // Index buffer view

    // Save model as glTF
    tinygltf::TinyGLTF gltf_writer;
    return gltf_writer.WriteGltfSceneToFile(&model, filename);
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.ply> <output.gltf>" << std::endl;
        return -1;
    }

    std::string input_pcd_file = argv[1];
    std::string output_ply_file = argv[2];
    std::string output_gltf_file = argv[3];

    // Load the PCD point cloud using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_file, *cloud) == -1) {
        std::cerr << "Error loading PCD file: " << input_pcd_file << std::endl;
        return -1;
    }

    std::cout << "Loaded " << cloud->points.size() << " points from " << input_pcd_file << std::endl;

    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setKSearch(20); // You can adjust this number based on your data
    normal_estimator.compute(*normals);

    // Combine the points and normals for CGAL input
    std::vector<PointVectorPair> points;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        Point_3 point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        Vector_3 normal(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
        points.push_back(std::make_pair(point, normal));
    }

    // Perform Poisson reconstruction
    Mesh mesh;
    double sm_angle = 20.0; // Min triangle angle in degrees
    double sm_radius = 30.0; // Max triangle size w.r.t. point cloud average spacing
    double sm_distance = 0.375; // Surface approximation error

    if (!CGAL::poisson_surface_reconstruction_delaunay(
        points.begin(), points.end(),
        CGAL::make_normal_of_point_with_normal_map(PointVectorPair()),
        mesh, sm_angle, sm_radius, sm_distance)) {
        std::cerr << "Poisson reconstruction failed!" << std::endl;
        return -1;
    }

    std::cout << "Poisson reconstruction succeeded, saving mesh to " << output_ply_file << std::endl;

    // Write the mesh to a PLY file
    std::ofstream out(output_ply_file);
    CGAL::IO::write_PLY(out, mesh);

    // Save the mesh to GLTF format
    if (!saveMeshAsGLTF(mesh, output_gltf_file)) {
        std::cerr << "Failed to save mesh as GLTF!" << std::endl;
        return -1;
    }

    std::cout << "Mesh saved as GLTF to " << output_gltf_file << std::endl;

    return 0;
}