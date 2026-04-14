import mujoco

urdf_path = "/home/fabian/Documents/TFM/models/softhandV2.urdf"
output_path = "/home/fabian/Documents/TFM/misc/mujoco-3.6.0/bin/softhandV2.xml"

# Cargar todas las mallas como assets
import os
assets = {}
meshes_dir = "/home/fabian/Documents/TFM/models/meshes"
for f in os.listdir(meshes_dir):
    with open(os.path.join(meshes_dir, f), 'rb') as fh:
        assets[f] = fh.read()

# Cargar y compilar el URDF
model = mujoco.MjModel.from_xml_path(urdf_path, assets)

# Guardar como XML de MuJoCo
mujoco.mj_saveLastXML(output_path, model)
print(f"Guardado en: {output_path}")
