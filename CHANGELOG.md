# Changelog

All notable changes to `robotiq_3f_description` are documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

### Added
- Initial ROS 1 (Noetic) catkin package created with `catkin_create_pkg`.
- `config/part_poses.yaml` — single source-of-truth for all link poses (mm + axis-angle).
- `meshes/collision/` — STL collision meshes for all 6 unique parts.
- `meshes/visual/` — STL + DAE visual meshes for all 6 unique parts (metres).
- `meshes/visual_in_mm/` — original CAD DAE exports in millimetres (reference).
- `urdf/robotiq_3f_gripper_macro.xacro` — reusable xacro macro with `prefix` and `parent` parameters.
- `urdf/robotiq_3f_gripper.urdf.xacro` — standalone top-level xacro for display/testing.
- `urdf/robotiq_3f_gripper.urdf` — pre-built flat URDF (generated from xacro).
- `launch/display.launch` — loads the URDF and opens RViz.
- `scripts/create_xacro_from_parts.py` — generates macro + standalone xacro from `part_poses.yaml`.
- `scripts/create_urdf_from_parts.py` — generates flat URDF from `part_poses.yaml`.
- `scripts/create_ply_from_parts.py` — assembles and previews the gripper in Open3D, saves PLY outputs.
- `README.md`, `.gitignore`, `CHANGELOG.md`.
