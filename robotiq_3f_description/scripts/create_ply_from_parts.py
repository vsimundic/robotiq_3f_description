#!/usr/bin/env python3
"""
Load part meshes (STL from collision/ and visual/) and assemble the Robotiq
3-Finger gripper model in Open3D using the poses and parent-link hierarchy
defined in part_poses.yaml.

Units in the YAML are millimetres & degrees; STL meshes are already in metres,
so positions are converted from mm → m before use.
"""

import os
import sys
import copy
import yaml
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
YAML_PATH = os.path.join(BASE_DIR, "config", "part_poses.yaml")
COLLISION_DIR = os.path.join(BASE_DIR, "meshes", "collision")
VISUAL_DIR = os.path.join(BASE_DIR, "meshes", "visual")
OUTPUT_DIR = os.path.join(BASE_DIR, "meshes", "assembled")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def euler_deg_to_rotation_matrix(euler_xyz_deg: list) -> np.ndarray:
    """Convert intrinsic XYZ Euler angles (degrees) → 3×3 rotation matrix."""
    r = Rotation.from_euler("zyx", euler_xyz_deg, degrees=True)
    return r.as_matrix()

def axis_angle_deg_to_rotation_matrix(axis_angle_deg: list) -> np.ndarray:
    """Convert axis-angle (degrees) → 3×3 rotation matrix."""
    axis = np.array(axis_angle_deg[:3], dtype=float)
    angle_deg = float(axis_angle_deg[3])
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or abs(angle_deg) < 1e-12:
        return np.eye(3)
    r = Rotation.from_rotvec(np.radians(angle_deg) * axis / norm)
    return r.as_matrix()

def pose_to_transform(position_mm: list, orientation_deg: list) -> np.ndarray:
    """Build a 4×4 homogeneous transform from position (mm) and orientation (deg)."""
    T = np.eye(4)
    # T[:3, :3] = euler_deg_to_rotation_matrix(orientation_deg)
    T[:3, :3] = axis_angle_deg_to_rotation_matrix(orientation_deg)
    T[:3, 3] = np.array(position_mm) * 1e-3  # mm → m
    return T


def load_mesh(part_filename: str, mesh_dir: str, extension: str = ".stl") -> o3d.geometry.TriangleMesh:
    """Load a triangle mesh from *mesh_dir*/<part_filename><extension>."""
    mesh_path = os.path.join(mesh_dir, part_filename + extension)
    if not os.path.isfile(mesh_path):
        print(f"[WARN] Mesh file not found: {mesh_path}")
        return None
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    if len(mesh.vertices) == 0:
        print(f"[WARN] Empty mesh: {mesh_path}")
        return None
    mesh.compute_vertex_normals()
    return mesh


# ---------------------------------------------------------------------------
# Build the assembled model
# ---------------------------------------------------------------------------
def compute_world_transforms(parts: dict) -> dict:
    """
    Walk the kinematic tree defined by *supposed_parent_link* and compute
    the world (absolute) 4×4 transform for every part.

    All poses in the YAML are given **relative to robotiq_3f_frame** (the
    global frame), so the world transform of each part is simply its own
    pose_to_transform.  The parent-link info is printed for validation.
    """
    world_transforms = {}

    for name, props in parts.items():
        T = pose_to_transform(props["position"], props["orientation"])
        world_transforms[name] = T

    return world_transforms


def assemble_gripper(
    parts: dict,
    mesh_dir: str,
    extension: str = ".stl",
    color=None,
):
    """
    Load every part mesh, transform it to its world pose, and return the
    list of transformed meshes ready for visualisation.
    """
    world_transforms = compute_world_transforms(parts)
    meshes = []

    for name, props in parts.items():
        part_filename = props["part_filename"]
        mesh = load_mesh(part_filename, mesh_dir, extension)
        if mesh is None:
            continue

        T = world_transforms[name]
        mesh.transform(T)

        if color is not None:
            mesh.paint_uniform_color(color)

        meshes.append(mesh)
        print(f"  ✓ {name:30s}  ←  {part_filename}{extension}  (parent: {props['supposed_parent_link']})")
        print(f"    position (m): {T[:3, 3].tolist()}")
        print(f"    orientation (deg): {props['orientation']}")
        print(f"    Transformation applied to {name}:\n{T}\n")

    return meshes


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parts = load_yaml(YAML_PATH)
    print(f"Loaded {len(parts)} part entries from {YAML_PATH}\n")

    # --- Collision meshes (STL) -------------------------------------------
    print("Loading collision meshes (.stl) …")
    collision_meshes = assemble_gripper(
        parts,
        mesh_dir=COLLISION_DIR,
        extension=".stl",
        color=[0.7, 0.7, 0.7],  # light grey
    )

    # --- Visual meshes (STL not available, try DAE via trimesh fallback) ---
    # Open3D cannot read .dae natively.  If the visual/ directory contains
    # .stl copies we use those; otherwise we skip the visual set.
    visual_meshes = []
    sample_visual = os.path.join(VISUAL_DIR, "AGS_palm.stl")
    if os.path.isfile(sample_visual):
        print("\nLoading visual meshes (.stl) …")
        visual_meshes = assemble_gripper(
            parts,
            mesh_dir=VISUAL_DIR,
            extension=".stl",
            color=[0.2, 0.5, 0.8],  # steel blue
        )
    else:
        print(
            "\n[INFO] Visual meshes are in .dae format which Open3D cannot "
            "read directly.  Only collision meshes will be shown.\n"
            "       To include visual meshes, convert them to .stl or .ply."
        )

    # --- Combine & visualise ------------------------------------------------
    all_meshes = collision_meshes + visual_meshes
    if not all_meshes:
        print("[ERROR] No meshes were loaded – nothing to display.")
        sys.exit(1)

    # Save a single combined PLY
    combined = o3d.geometry.TriangleMesh()
    for m in all_meshes:
        combined += m
    combined.compute_vertex_normals()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    out_ply = os.path.join(OUTPUT_DIR, "robotiq_3f_assembled.ply")
    o3d.io.write_triangle_mesh(out_ply, combined)
    print(f"\nCombined mesh saved to {out_ply}")
    print(f"  vertices : {len(combined.vertices)}")
    print(f"  triangles: {len(combined.triangles)}")

    # --- Add a coordinate frame for reference --------------------------------
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

    print("\nOpening Open3D viewer …")
    o3d.visualization.draw_geometries(
        [origin] + all_meshes,
        window_name="Robotiq 3-Finger Gripper",
        width=1280,
        height=720,
    )


    # Save collision and visual meshes
    collision_out_ply = os.path.join(OUTPUT_DIR, "robotiq_3f_collision.ply")
    visual_out_ply = os.path.join(OUTPUT_DIR, "robotiq_3f_visual.ply")
    full_collision_mesh = o3d.geometry.TriangleMesh()
    for m in collision_meshes:
        full_collision_mesh += m
    full_collision_mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(collision_out_ply, full_collision_mesh)
    print(f"Collision mesh saved to {collision_out_ply}")
    if visual_meshes:
        full_visual_mesh = o3d.geometry.TriangleMesh()
        for m in visual_meshes:
            full_visual_mesh += m
        full_visual_mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(visual_out_ply, full_visual_mesh)
        print(f"Visual mesh saved to {visual_out_ply}")

if __name__ == "__main__":
    main()
