#!/usr/bin/env python3
"""
Generate xacro files for the Robotiq 3-Finger Adaptive Gripper from part_poses.yaml.

Two files are written into the urdf/ directory:
  - robotiq_3f_gripper_macro.xacro   reusable macro (params: prefix, parent)
  - robotiq_3f_gripper.urdf.xacro    standalone top-level file (attaches to "world")

The same transform maths as create_urdf_from_parts.py are used: poses in the YAML
are absolute w.r.t. robotiq_3f_frame, and relative parent→child transforms are
computed for each joint origin.
"""

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from xml.dom import minidom
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, os.pardir))
YAML_PATH = os.path.join(BASE_DIR, "config", "part_poses.yaml")
URDF_DIR = os.path.join(BASE_DIR, "urdf")
ROS_PACKAGE_NAME = "robotiq_3f_description"

# ---------------------------------------------------------------------------
# Maths helpers  (identical to create_urdf_from_parts.py)
# ---------------------------------------------------------------------------

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
    T = np.eye(4)
    T[:3, :3] = axis_angle_deg_to_rotation(orientation_deg).as_matrix()
    T[:3, 3] = np.array(position_mm, dtype=float) * 1e-3
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    T_inv = np.eye(4)
    R, t = T[:3, :3], T[:3, 3]
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def transform_to_xyz_rpy(T: np.ndarray):
    xyz = T[:3, 3].tolist()
    rpy = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=False).tolist()
    return xyz, rpy


def fmt(values, precision=8, zero_thresh=1e-10) -> str:
    cleaned = [0.0 if abs(v) < zero_thresh else v for v in values]
    return " ".join(f"{v:.{precision}g}" for v in cleaned)

# ---------------------------------------------------------------------------
# Pre-compute world transforms for every part
# ---------------------------------------------------------------------------

def compute_relative_joints(parts: dict) -> dict:
    """
    Return a dict  name → {parent, part_filename, xyz_str, rpy_str}
    with the relative (parent→child) joint origin already formatted.
    """
    root = "robotiq_3f_frame"
    world_T = {root: np.eye(4)}
    for name, props in parts.items():
        world_T[name] = pose_to_transform(props["position"], props["orientation"])

    joints = {}
    for name, props in parts.items():
        parent = props["supposed_parent_link"]
        T_rel = invert_transform(world_T.get(parent, np.eye(4))) @ world_T[name]
        xyz, rpy = transform_to_xyz_rpy(T_rel)
        joints[name] = {
            "parent":        parent,
            "part_filename": props["part_filename"],
            "xyz":           fmt(xyz),
            "rpy":           fmt(rpy),
        }
    return joints

# ---------------------------------------------------------------------------
# XML helpers
# ---------------------------------------------------------------------------

def _link_block(parent_elem: ET.Element, link_name: str, part_filename: str,
                prefix_expr: str = ""):
    """Append a <link> element with visual, collision and inertial sub-elements."""
    link = ET.SubElement(parent_elem, "link", name=f"{prefix_expr}{link_name}")

    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    vis_geom = ET.SubElement(visual, "geometry")
    ET.SubElement(vis_geom, "mesh",
                  filename=f"package://{ROS_PACKAGE_NAME}/meshes/visual/{part_filename}.stl")

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    col_geom = ET.SubElement(collision, "geometry")
    ET.SubElement(col_geom, "mesh",
                  filename=f"package://{ROS_PACKAGE_NAME}/meshes/collision/{part_filename}.stl")

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", value="0.01")
    ET.SubElement(inertial, "inertia",
                  ixx="1e-6", ixy="0", ixz="0",
                  iyy="1e-6", iyz="0", izz="1e-6")


def _joint_block(parent_elem: ET.Element, child_name: str, parent_name: str,
                 xyz: str, rpy: str, prefix_expr: str = ""):
    """Append a fixed <joint> element."""
    joint = ET.SubElement(parent_elem, "joint",
                          name=f"{prefix_expr}{parent_name}_to_{child_name}_joint",
                          type="fixed")
    ET.SubElement(joint, "parent", link=f"{prefix_expr}{parent_name}")
    ET.SubElement(joint, "child",  link=f"{prefix_expr}{child_name}")
    ET.SubElement(joint, "origin", xyz=xyz, rpy=rpy)


def prettify(elem: ET.Element) -> str:
    rough = ET.tostring(elem, encoding="unicode")
    dom = minidom.parseString(rough)
    # minidom adds an XML declaration – keep it
    return dom.toprettyxml(indent="  ", encoding=None)

# ---------------------------------------------------------------------------
# Build macro xacro
# ---------------------------------------------------------------------------

def build_macro_xacro(joints: dict) -> str:
    """
    Build the content of robotiq_3f_gripper_macro.xacro.
    Uses xacro ${prefix} substitutions for all link/joint names.
    """
    # We use a plain string builder for the macro body because ElementTree
    # cannot represent xacro namespace attributes cleanly.  The link/joint
    # blocks are written as indented XML text.

    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"'
                 ' name="robotiq_3f_gripper">')
    lines.append('  <!--')
    lines.append('    Robotiq 3-Finger Adaptive Gripper – xacro macro.')
    lines.append('    AUTO-GENERATED by create_xacro_from_parts.py – do not edit by hand.')
    lines.append('')
    lines.append('    Parameters:')
    lines.append('      prefix  – optional namespace prefix for all links/joints (default: "")')
    lines.append('      parent  – parent link to attach the gripper to')
    lines.append('')
    lines.append('    Usage:')
    lines.append(f'      <xacro:include filename="$(find {ROS_PACKAGE_NAME})'
                 '/urdf/robotiq_3f_gripper_macro.xacro"/>')
    lines.append('      <xacro:robotiq_3f_gripper prefix="gripper_" parent="tool0"/>')
    lines.append('  -->')
    lines.append('')
    lines.append("  <xacro:macro name=\"robotiq_3f_gripper\" params=\"prefix:='' parent\">")
    lines.append('')

    # Root frame + attachment joint
    lines.append('    <!-- Root frame -->')
    lines.append('    <link name="${prefix}robotiq_3f_frame"/>')
    lines.append('')
    lines.append('    <joint name="${prefix}${parent}_to_robotiq_3f_frame_joint" type="fixed">')
    lines.append('      <parent link="${parent}"/>')
    lines.append('      <child  link="${prefix}robotiq_3f_frame"/>')
    lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    lines.append('    </joint>')
    lines.append('')

    # One block per part
    for name, j in joints.items():
        parent_link = j["parent"]
        part_filename = j["part_filename"]
        xyz = j["xyz"]
        rpy = j["rpy"]

        lines.append(f'    <!-- {name} -->')
        lines.append(f'    <link name="${"{prefix}"}{name}">')
        lines.append('      <visual>')
        lines.append('        <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append('        <geometry>')
        lines.append(f'          <mesh filename="package://{ROS_PACKAGE_NAME}'
                     f'/meshes/visual/{part_filename}.stl"/>')
        lines.append('        </geometry>')
        lines.append('      </visual>')
        lines.append('      <collision>')
        lines.append('        <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append('        <geometry>')
        lines.append(f'          <mesh filename="package://{ROS_PACKAGE_NAME}'
                     f'/meshes/collision/{part_filename}.stl"/>')
        lines.append('        </geometry>')
        lines.append('      </collision>')
        lines.append('      <inertial>')
        lines.append('        <mass value="0.01"/>')
        lines.append('        <inertia ixx="1e-6" ixy="0" ixz="0"'
                     ' iyy="1e-6" iyz="0" izz="1e-6"/>')
        lines.append('      </inertial>')
        lines.append('    </link>')
        lines.append('')
        lines.append(f'    <joint name="${"{prefix}"}{parent_link}_to_{name}_joint"'
                     ' type="fixed">')
        lines.append(f'      <parent link="${"{prefix}"}{parent_link}"/>')
        lines.append(f'      <child  link="${"{prefix}"}{name}"/>')
        lines.append(f'      <origin xyz="{xyz}" rpy="{rpy}"/>')
        lines.append('    </joint>')
        lines.append('')

        print(f"  ✓ {name:30s}  parent={parent_link:30s}  xyz=[{xyz}]  rpy=[{rpy}]")

    lines.append('  </xacro:macro>')
    lines.append('</robot>')
    return "\n".join(lines) + "\n"

# ---------------------------------------------------------------------------
# Build standalone top-level xacro
# ---------------------------------------------------------------------------

def build_toplevel_xacro() -> str:
    lines = []
    lines.append('<?xml version="1.0"?>')
    lines.append('<!--')
    lines.append('  Standalone xacro for the Robotiq 3-Finger Adaptive Gripper.')
    lines.append('  AUTO-GENERATED by create_xacro_from_parts.py – do not edit by hand.')
    lines.append('')
    lines.append('  This file includes the macro definition and instantiates the gripper')
    lines.append('  attached to a "world" link. Useful for previewing and testing.')
    lines.append('')
    lines.append('  Process with:')
    lines.append(f'    xacro $(find {ROS_PACKAGE_NAME})/urdf/robotiq_3f_gripper.urdf.xacro'
                 ' > /tmp/robotiq_3f_gripper.urdf')
    lines.append('-->')
    lines.append('<robot xmlns:xacro="http://www.ros.org/wiki/xacro"'
                 ' name="robotiq_3f_gripper">')
    lines.append('')
    lines.append(f'  <xacro:include filename="$(find {ROS_PACKAGE_NAME})'
                 '/urdf/robotiq_3f_gripper_macro.xacro"/>')
    lines.append('')
    lines.append('  <!-- Standalone root link -->')
    lines.append('  <link name="world"/>')
    lines.append('')
    lines.append('  <!-- Instantiate the gripper attached to "world" -->')
    lines.append('  <xacro:robotiq_3f_gripper prefix="" parent="world"/>')
    lines.append('')
    lines.append('</robot>')
    return "\n".join(lines) + "\n"

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parts = load_yaml(YAML_PATH)
    print(f"Loaded {len(parts)} part entries from {YAML_PATH}\n")

    joints = compute_relative_joints(parts)

    # --- macro xacro ---
    macro_path = os.path.join(URDF_DIR, "robotiq_3f_gripper_macro.xacro")
    print("Building robotiq_3f_gripper_macro.xacro …")
    macro_content = build_macro_xacro(joints)
    with open(macro_path, "w") as f:
        f.write(macro_content)
    print(f"  → {macro_path}  ({len(joints)} links)\n")

    # --- standalone xacro ---
    toplevel_path = os.path.join(URDF_DIR, "robotiq_3f_gripper.urdf.xacro")
    print("Building robotiq_3f_gripper.urdf.xacro …")
    with open(toplevel_path, "w") as f:
        f.write(build_toplevel_xacro())
    print(f"  → {toplevel_path}\n")

    print("Done.  To regenerate the flat URDF run:")
    print(f"  rosrun {ROS_PACKAGE_NAME} create_urdf_from_parts.py")
    print("  or:")
    print(f"  xacro $(find {ROS_PACKAGE_NAME})/urdf/robotiq_3f_gripper.urdf.xacro"
          " > $(find {ROS_PACKAGE_NAME})/urdf/robotiq_3f_gripper.urdf")


if __name__ == "__main__":
    main()
