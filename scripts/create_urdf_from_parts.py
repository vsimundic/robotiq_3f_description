#!/usr/bin/env python3
"""
Generate a URDF for the Robotiq 3-Finger Adaptive Gripper from part_poses.yaml.

Poses in the YAML are relative to the root frame (robotiq_3f_frame), so
relative parent-to-child transforms are computed for each URDF joint.
Orientations are axis-angle [x, y, z, deg]; positions are in mm (converted to m).
Mesh URIs use: package://robotiq_3f_description/meshes/{collision,visual}/
"""

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET
from xml.dom import minidom

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
YAML_PATH = os.path.join(BASE_DIR, "config", "part_poses.yaml")
URDF_DIR = os.path.join(BASE_DIR, "urdf")
ROS_PACKAGE_NAME = "robotiq_3f_description"


def load_yaml(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def axis_angle_deg_to_rotation(axis_angle_deg: list) -> Rotation:
    """Convert [x, y, z, deg] axis-angle to a scipy Rotation."""
    axis = np.array(axis_angle_deg[:3], dtype=float)
    angle_deg = float(axis_angle_deg[3])
    norm = np.linalg.norm(axis)
    if norm < 1e-12 or abs(angle_deg) < 1e-12:
        return Rotation.identity()
    return Rotation.from_rotvec(np.radians(angle_deg) * axis / norm)


def pose_to_transform(position_mm: list, orientation_deg: list) -> np.ndarray:
    """Build a 4x4 homogeneous transform from position (mm) and axis-angle (deg)."""
    T = np.eye(4)
    T[:3, :3] = axis_angle_deg_to_rotation(orientation_deg).as_matrix()
    T[:3, 3] = np.array(position_mm, dtype=float) * 1e-3
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Return the inverse of a 4x4 homogeneous transform."""
    T_inv = np.eye(4)
    R, t = T[:3, :3], T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def transform_to_xyz_rpy(T: np.ndarray):
    """Extract (x, y, z) and (roll, pitch, yaw) in radians from a 4x4 transform."""
    xyz = T[:3, 3].tolist()
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=False).tolist()
    return xyz, rpy


def fmt(values, precision=8, zero_thresh=1e-10):
    """Format floats as a space-separated string, snapping near-zero values to 0."""
    cleaned = [0.0 if abs(v) < zero_thresh else v for v in values]
    return " ".join(f"{v:.{precision}g}" for v in cleaned)


def mesh_uri(mesh_type: str, part_filename: str, use_abs_paths: bool) -> str:
    """Return the mesh filename attribute for a visual or collision element."""
    subdir = "visual" if mesh_type == "visual" else "collision"
    if use_abs_paths:
        return os.path.join(BASE_DIR, "meshes", subdir, f"{part_filename}.stl")
    return f"package://{ROS_PACKAGE_NAME}/meshes/{subdir}/{part_filename}.stl"


def build_urdf(parts: dict, use_abs_paths: bool = False) -> ET.Element:
    """Build an ElementTree representing the full URDF of the gripper."""
    robot = ET.Element("robot", name="robotiq_3f_gripper")

    root_link_name = "robotiq_3f_frame"
    ET.SubElement(robot, "link", name=root_link_name)

    world_transforms = {root_link_name: np.eye(4)}
    for name, props in parts.items():
        world_transforms[name] = pose_to_transform(props["position"], props["orientation"])

    for name, props in parts.items():
        parent_name = props["supposed_parent_link"]
        part_filename = props["part_filename"]

        T_rel = invert_transform(world_transforms.get(parent_name, np.eye(4))) @ world_transforms[name]
        xyz, rpy = transform_to_xyz_rpy(T_rel)

        link_elem = ET.SubElement(robot, "link", name=name)

        visual = ET.SubElement(link_elem, "visual")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        vis_geom = ET.SubElement(visual, "geometry")
        ET.SubElement(vis_geom, "mesh", filename=mesh_uri("visual", part_filename, use_abs_paths))

        collision = ET.SubElement(link_elem, "collision")
        ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        col_geom = ET.SubElement(collision, "geometry")
        ET.SubElement(col_geom, "mesh", filename=mesh_uri("collision", part_filename, use_abs_paths))

        inertial = ET.SubElement(link_elem, "inertial")
        ET.SubElement(inertial, "mass", value="0.01")
        ET.SubElement(inertial, "inertia",
                      ixx="1e-6", ixy="0", ixz="0",
                      iyy="1e-6", iyz="0", izz="1e-6")

        joint_elem = ET.SubElement(robot, "joint",
                                   name=f"{parent_name}_to_{name}_joint", type="fixed")
        ET.SubElement(joint_elem, "parent", link=parent_name)
        ET.SubElement(joint_elem, "child", link=name)
        ET.SubElement(joint_elem, "origin", xyz=fmt(xyz), rpy=fmt(rpy))

        print(f"  ✓ {name:30s}  parent={parent_name:30s}  "
              f"xyz=[{fmt(xyz)}]  rpy=[{fmt(rpy)}]")

    return robot


def prettify_xml(elem: ET.Element) -> str:
    """Return a pretty-printed XML string for the Element."""
    rough = ET.tostring(elem, encoding="unicode")
    return minidom.parseString(rough).toprettyxml(indent="  ", encoding=None)


def write_urdf(parts: dict, out_path: str, use_abs_paths: bool = False):
    """Build the URDF, write it to *out_path*, and print a summary."""
    label = "absolute paths" if use_abs_paths else "package:// URIs"
    print(f"Building URDF ({label}) …")
    robot_elem = build_urdf(parts, use_abs_paths=use_abs_paths)

    with open(out_path, "w") as f:
        f.write(prettify_xml(robot_elem))

    print(f"  → {out_path}")
    print(f"    Links : {len(robot_elem.findall('link'))}")
    print(f"    Joints: {len(robot_elem.findall('joint'))}\n")


def main():
    parts = load_yaml(YAML_PATH)
    print(f"Loaded {len(parts)} part entries from {YAML_PATH}\n")

    write_urdf(parts, os.path.join(URDF_DIR, "robotiq_3f_gripper.urdf"),
               use_abs_paths=False)
    write_urdf(parts, os.path.join(URDF_DIR, "robotiq_3f_gripper_abs_paths.urdf"),
               use_abs_paths=True)


if __name__ == "__main__":
    main()
