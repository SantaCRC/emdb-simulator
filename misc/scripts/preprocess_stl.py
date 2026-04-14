import os
import trimesh

meshes_dir = "/home/fabian/Documents/TFM/models/meshes"

for filename in os.listdir(meshes_dir):
    if filename.lower().endswith(".stl"):
        filepath = os.path.join(meshes_dir, filename)
        out_path = filepath.replace(".stl", ".obj").replace(".STL", ".obj")
        try:
            mesh = trimesh.load(filepath, force='mesh')
            mesh.export(out_path)
            print(f"OK: {filename} → {os.path.basename(out_path)}")
        except Exception as e:
            print(f"ERROR en {filename}: {e}")
