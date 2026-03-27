import numpy as np
import open3d as o3d
import time
import os

########################################################################################
### Adjust spheres in spheres.txt and save to see live updates in the visualization. ###
########################################################################################

# Create a mesh for the tool and add it to the list for visualization
base_dir = '/home/RVLuser/data/Robotiq3Finger/one_finger/robotiq_new_mesh'
gripper_3f_mesh = o3d.io.read_triangle_mesh(os.path.join(base_dir, 'mesh.ply')) # original mesh
spheres_txt_path = os.path.join(base_dir, 'spheres.txt')
save_npy_path = os.path.join(base_dir, 'spheres.npy')


gripper_3f_mesh.compute_vertex_normals()

def load_spheres_from_txt(filepath, convert_to_meters=True):
    """Loads spheres from a csv-like txt file, returns scaled array."""
    try:
        # loadtxt will handle comma-separated rows, ignoring newlines and empty lines
        spheres = np.loadtxt(filepath, delimiter=',')
        if spheres.ndim == 1:
            spheres = spheres[np.newaxis, :]
        if convert_to_meters:
            return spheres / 1000.
        return spheres
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None

def create_sphere_geometry(spheres):
    """Creates a list of LineSet objects for the spheres."""
    sphere_meshes_ls = []
    for i in range(spheres.shape[0]):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=spheres[i, 3])
        sphere_ls = o3d.geometry.LineSet.create_from_triangle_mesh(sphere)
        sphere_ls.paint_uniform_color([0.8, 0, 0])
        vec = np.array(spheres[i, 0:3]).ravel()
        sphere_ls.translate(vec)
        sphere_meshes_ls.append(sphere_ls)
    return sphere_meshes_ls

last_mtime = 0

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(gripper_3f_mesh)

current_sphere_geos = []

print("Visualizing... You can edit spheres.txt and save to update live.")

while True:
    try:
        current_mtime = os.path.getmtime(spheres_txt_path)
    except OSError:
        current_mtime = last_mtime
        
    if current_mtime > last_mtime:
        print("Spheres file updated, reloading...")
        last_mtime = current_mtime
        
        # Remove old sphere geometries
        for geo in current_sphere_geos:
            vis.remove_geometry(geo, reset_bounding_box=False)
            
        # Load and add new ones
        spheres = load_spheres_from_txt(spheres_txt_path)
        if spheres is not None:
            current_sphere_geos = create_sphere_geometry(spheres)
            for geo in current_sphere_geos:
                vis.add_geometry(geo, reset_bounding_box=False)
                
    if not vis.poll_events():
        break
    vis.update_renderer()
    time.sleep(0.05)

vis.destroy_window()

spheres = load_spheres_from_txt(spheres_txt_path, convert_to_meters=False)
if spheres is not None:
    np.save(save_npy_path, spheres)
    print(f"Spheres saved to {save_npy_path}")