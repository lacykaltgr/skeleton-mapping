# write script which takes a mesh file as input argument
# reads the mesh file
# converts the mesh to point cloud
# saves the point cloud to a file at the same place, but with _cloud attached to the name

import open3d as o3d
import argparse
import os

def mesh2cloud(mesh_path):
    # it is a normal mesh, not triangle mesh
    # do not use read_triangle_mesh
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    pcd = mesh.sample_points_poisson_disk(100000)
    cloud_path = mesh_path.replace(".ply", "_cloud.pcd")
    # write as .pcd file
    o3d.io.write_point_cloud(cloud_path, pcd)
    print(f"Point cloud saved to {cloud_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Converts a mesh file to a point cloud file')
    parser.add_argument('mesh_path', type=str,
                        help='Path to the mesh file')
    args = parser.parse_args()
    mesh2cloud(args.mesh_path)