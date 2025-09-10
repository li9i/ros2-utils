#!/usr/bin/env python3
import os
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Open3D must be installed in your environment (pip install open3d)
import open3d as o3d


def cloud_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert ROS2 PointCloud2 to (N,3) numpy array."""
    pts = np.asarray([
        (x, y, z)
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ], dtype=np.float64)
    return pts


def numpy_to_o3d_pointcloud(points: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def preprocess_pointcloud(
    pcd: o3d.geometry.PointCloud,
    voxel_downsample_size: float,
    remove_statistical_outliers: bool,
    nso_nb_neighbors: int,
    nso_std_ratio: float,
) -> o3d.geometry.PointCloud:
    if voxel_downsample_size and voxel_downsample_size > 0.0:
        pcd = pcd.voxel_down_sample(voxel_downsample_size)

    if remove_statistical_outliers:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=nso_nb_neighbors, std_ratio=nso_std_ratio
        )
    return pcd


def estimate_normals(
    pcd: o3d.geometry.PointCloud, normal_radius: float, normal_max_nn: int
):
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=normal_radius, max_nn=normal_max_nn
        )
    )
    pcd.orient_normals_consistent_tangent_plane(10)


def poisson_reconstruction(
    pcd: o3d.geometry.PointCloud, depth: int, density_quantile: float
) -> o3d.geometry.TriangleMesh:
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth
    )
    if density_quantile is not None and 0.0 < density_quantile < 1.0:
        densities = np.asarray(densities)
        mask = densities < np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(mask)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    return mesh


def bpa_reconstruction(
    pcd: o3d.geometry.PointCloud, radii: List[float], tri_angle_deg: float
) -> o3d.geometry.TriangleMesh:
    # Open3D BPA uses radii list; ensure ascending and >0
    radii = sorted([r for r in radii if r > 0.0])
    if not radii:
        raise ValueError("bpa_radii must contain positive values")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )
    # Optional: simplify very sharp triangles if angle threshold provided
    if tri_angle_deg is not None:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=1)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    return mesh


def save_mesh(mesh: o3d.geometry.TriangleMesh, out_dir: str, basename: str, fmt: str) -> str:
    os.makedirs(out_dir, exist_ok=True)
    fmt = fmt.lower()
    if fmt not in ("obj", "ply"):
        raise ValueError('output_format must be "obj" or "ply"')
    path = os.path.join(out_dir, f"{basename}.{fmt}")
    o3d.io.write_triangle_mesh(path, mesh)
    return path


class PointCloudToMeshNode(Node):
    def __init__(self):
        super().__init__("pointcloud_to_ply_node")

        # Parameters (declare with defaults; override via YAML)
        self.declare_parameter("pointcloud_topic", "/rc_viscore/points2")
        self.declare_parameter("output_path", "/tmp/mesh_output")
        self.declare_parameter("output_basename", "mesh")
        self.declare_parameter("output_format", "obj")

        self.declare_parameter("voxel_downsample_size", 0.0)
        self.declare_parameter("remove_statistical_outliers", True)
        self.declare_parameter("nso_nb_neighbors", 20)
        self.declare_parameter("nso_std_ratio", 2.0)

        self.declare_parameter("normal_radius", 0.05)
        self.declare_parameter("normal_max_nn", 30)

        self.declare_parameter("reconstruction_method", "poisson")
        self.declare_parameter("poisson_depth", 9)
        self.declare_parameter("poisson_density_quantile", 0.05)
        self.declare_parameter("bpa_radii", [0.02, 0.04, 0.08])
        self.declare_parameter("bpa_triangle_angle", 30.0)

        self.declare_parameter("shutdown_after_save", True)
        self.declare_parameter("log_level_debug", False)

        topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value

        # Should match the topic's qos profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            PointCloud2, topic, self._callback, qos
        )
        self._done = False

        self.get_logger().info(f"Subscribed to {topic}")

    def _callback(self, msg: PointCloud2):
        if self._done:
            return
        self._done = True

        dbg = self.get_parameter("log_level_debug").value
        try:
            self.get_logger().info("Received point cloud; converting to numpy...")
            points = cloud_to_numpy(msg)
            self.get_logger().info(f"Point count: {points.shape[0]}")

            pcd = numpy_to_o3d_pointcloud(points)

            # Preprocess
            pcd = preprocess_pointcloud(
                pcd=pcd,
                voxel_downsample_size=self.get_parameter("voxel_downsample_size").value,
                remove_statistical_outliers=self.get_parameter("remove_statistical_outliers").value,
                nso_nb_neighbors=self.get_parameter("nso_nb_neighbors").value,
                nso_std_ratio=self.get_parameter("nso_std_ratio").value,
            )

            # Normals
            estimate_normals(
                pcd,
                self.get_parameter("normal_radius").value,
                self.get_parameter("normal_max_nn").value,
            )

            # Reconstruction method
            method = self.get_parameter("reconstruction_method").value.lower()
            if method == "poisson":
                depth = int(self.get_parameter("poisson_depth").value)
                dq = float(self.get_parameter("poisson_density_quantile").value)
                mesh = poisson_reconstruction(pcd, depth, dq)
                self.get_logger().info(f"Poisson reconstruction complete (depth={depth})")
            elif method == "bpa":
                radii = list(self.get_parameter("bpa_radii").value)
                tri_angle = float(self.get_parameter("bpa_triangle_angle").value)
                mesh = bpa_reconstruction(pcd, radii, tri_angle)
                self.get_logger().info(f"BPA reconstruction complete (radii={radii})")
            else:
                raise ValueError('reconstruction_method must be "poisson" or "bpa"')

            out_dir = self.get_parameter("output_path").value
            basename = self.get_parameter("output_basename").value
            fmt = self.get_parameter("output_format").value
            out_path = save_mesh(mesh, out_dir, basename, fmt)
            self.get_logger().info(f"Saved mesh to: {out_path}")

        except Exception as e:
            self.get_logger().error(f"Failed to create/save mesh: {e}")
        finally:
            if self.get_parameter("shutdown_after_save").value:
                self.get_logger().info("Shutting down node.")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToMeshNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
