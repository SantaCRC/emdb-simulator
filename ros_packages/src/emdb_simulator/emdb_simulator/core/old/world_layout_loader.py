import os
import xml.etree.ElementTree as ET
import yaml


class WorldLayoutLoader:
    def __init__(self, world_yaml_path: str):
        self.world_yaml_path = os.path.abspath(world_yaml_path)

        if not os.path.exists(self.world_yaml_path):
            raise FileNotFoundError(f"World file not found: {self.world_yaml_path}")

        with open(self.world_yaml_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)

    def apply(self, root: ET.Element, asset: ET.Element, worldbody: ET.Element):
        self._add_textures(asset)
        self._add_materials(asset)
        self._add_room(worldbody)

    def _add_textures(self, asset):
        for tex in self.config.get("textures", []):
            tex_attrs = {
                "name": tex["name"],
                "type": tex.get("type", "2d"),
            }

            if "builtin" in tex:
                tex_attrs["builtin"] = tex["builtin"]
            if "width" in tex:
                tex_attrs["width"] = str(tex["width"])
            if "height" in tex:
                tex_attrs["height"] = str(tex["height"])
            if "rgb1" in tex:
                tex_attrs["rgb1"] = self._vec(tex["rgb1"])
            if "rgb2" in tex:
                tex_attrs["rgb2"] = self._vec(tex["rgb2"])
            if "file" in tex:
                tex_attrs["file"] = tex["file"]

            ET.SubElement(asset, "texture", **tex_attrs)

    def _add_materials(self, asset):
        for mat in self.config.get("materials", []):
            mat_attrs = {"name": mat["name"]}

            if "texture" in mat:
                mat_attrs["texture"] = mat["texture"]
            if "texrepeat" in mat:
                mat_attrs["texrepeat"] = self._vec(mat["texrepeat"])
            if "reflectance" in mat:
                mat_attrs["reflectance"] = str(mat["reflectance"])
            if "shininess" in mat:
                mat_attrs["shininess"] = str(mat["shininess"])
            if "specular" in mat:
                mat_attrs["specular"] = str(mat["specular"])
            if "texuniform" in mat:
                mat_attrs["texuniform"] = str(mat["texuniform"]).lower()

            ET.SubElement(asset, "material", **mat_attrs)

    def _add_room(self, worldbody):
        room = self.config.get("room", {})

        for floor in room.get("floor", []):
            self._add_box_geom(worldbody, floor, rgba="0.8 0.8 0.8 1")

        for wall in room.get("walls", []):
            self._add_box_geom(worldbody, wall, rgba="0.9 0.9 0.9 1")

    def _add_box_geom(self, parent, cfg, rgba="0.7 0.7 0.7 1"):
        geom_attrs = {
            "name": cfg["name"],
            "type": "box",
            "size": self._vec(cfg["size"]),
            "pos": self._vec(cfg["pos"]),
        }

        if "material" in cfg:
            geom_attrs["material"] = cfg["material"]
        else:
            geom_attrs["rgba"] = self._rgba(cfg.get("rgba"), default=rgba)

        ET.SubElement(parent, "geom", **geom_attrs)

    @staticmethod
    def _vec(values):
        return " ".join(str(v) for v in values)

    @staticmethod
    def _rgba(values, default="0.7 0.7 0.7 1"):
        if values is None:
            return default
        return " ".join(str(v) for v in values)