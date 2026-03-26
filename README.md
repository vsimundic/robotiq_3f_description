# robotiq_3f_description

A ROS 1 (Noetic) description package for the **Robotiq 3-Finger Adaptive Gripper (AGS)**.  
Contains meshes, URDF/xacro files, and helper scripts to regenerate everything from a single source-of-truth YAML file.

---

## Package layout

```
robotiq_3f_description/
├── config/
│   └── part_poses.yaml          ← single source of truth: positions (mm) + axis-angle orientations (deg)
├── launch/
│   └── display.launch           ← visualise in RViz
├── meshes/
│   ├── collision/               ← STL files used for collision geometry (metres)
│   ├── visual/                  ← STL + DAE files used for visual geometry (metres)
│   └── visual_in_mm/            ← original CAD exports in millimetres (reference only)
├── scripts/
│   ├── create_xacro_from_parts.py   ← regenerates the macro + standalone xacro from part_poses.yaml
│   ├── create_urdf_from_parts.py    ← regenerates the flat URDF from part_poses.yaml
│   └── create_ply_from_parts.py     ← assembles & previews the gripper in Open3D
├── urdf/
│   ├── robotiq_3f_gripper_macro.xacro   ← reusable xacro macro (prefix + parent params)
│   ├── robotiq_3f_gripper.urdf.xacro    ← standalone top-level xacro (attaches to "world")
│   └── robotiq_3f_gripper.urdf          ← pre-built flat URDF (generated, do not edit by hand)
├── CMakeLists.txt
└── package.xml
```

---

## Dependencies

| Dependency | Purpose |
|---|---|
| `xacro` | Process xacro files |
| `urdf` | URDF parsing |
| `robot_state_publisher` | Publish TF from URDF |
| `joint_state_publisher` | Publish joint states for RViz |
| `rviz` | Visualisation |
| `scipy`, `numpy` | Transform maths in the Python scripts |
| `open3d` | Mesh assembly & preview (`create_ply_from_parts.py` only) |

Install ROS deps with rosdep:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## Building

```bash
cd <your_ws>
catkin_make
source devel/setup.bash
```

---

## Usage

### Visualise in RViz

```bash
roslaunch robotiq_3f_description display.launch
```

### Use the gripper macro in another robot description

```xml
<xacro:include filename="$(find robotiq_3f_description)/urdf/robotiq_3f_gripper_macro.xacro"/>
<xacro:robotiq_3f_gripper prefix="gripper_" parent="tool0"/>
```

---

## Updating after changing `part_poses.yaml`

All URDF and xacro files are **generated** — edit only `config/part_poses.yaml`, then run the scripts in order:

```bash
cd src/robotiq_3f_description

# 1. Regenerate macro xacro + standalone xacro
python3 scripts/create_xacro_from_parts.py

# 2. Regenerate flat URDF
python3 scripts/create_urdf_from_parts.py

# 3. (Optional) Preview assembled mesh in Open3D
python3 scripts/create_ply_from_parts.py
```

### `part_poses.yaml` format

```yaml
<link_name>:
  position: [x_mm, y_mm, z_mm]          # position relative to robotiq_3f_frame, in mm
  orientation: [ax, ay, az, angle_deg]   # axis-angle, axis need not be normalised
  part_filename: "<mesh_stem>"           # filename without extension in meshes/{visual,collision}/
  supposed_parent_link: "<parent_name>"  # parent link in the kinematic tree
```

---

## License

BSD
