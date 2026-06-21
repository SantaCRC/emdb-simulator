import os
import xml.etree.ElementTree as ET
import yaml
import random


class ObjectLoader:
    def __init__(self, objects_yaml_path: str, rng=None):
        self.objects_yaml_path = os.path.abspath(objects_yaml_path)

        if not os.path.exists(self.objects_yaml_path):
            raise FileNotFoundError(f"Objects file not found: {self.objects_yaml_path}")

        with open(self.objects_yaml_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f) or {}

        self.rng = rng if rng is not None else random.Random()

    def apply(self, root: ET.Element, asset: ET.Element, worldbody: ET.Element):
        for obj_cfg in self.config.get("objects", []):
            self._add_object(worldbody, obj_cfg)

    def _add_object(self, worldbody: ET.Element, cfg: dict):
        pos = self._resolve_position(cfg.get("placement", {}))

        body_attrs = {
            "name": cfg["name"],
            "pos": self._vec(pos),
        }
        body = ET.SubElement(worldbody, "body", **body_attrs)

        if cfg.get("freejoint", True):
            ET.SubElement(body, "freejoint")

        geom_attrs = {
            "name": f'{cfg["name"]}_geom',
            "type": cfg["type"],
            "size": self._vec(cfg["size"]),
        }

        if "material" in cfg:
            geom_attrs["material"] = cfg["material"]
        else:
            geom_attrs["rgba"] = self._rgba(cfg.get("rgba", [0.7, 0.7, 0.7, 1.0]))

        if "density" in cfg:
            geom_attrs["density"] = str(cfg["density"])
        if "friction" in cfg:
            geom_attrs["friction"] = self._vec(cfg["friction"])

        ET.SubElement(body, "geom", **geom_attrs)

    def _resolve_position(self, placement: dict):
        mode = placement.get("mode", "fixed")

        if mode == "fixed":
            return placement["pos"]

        if mode == "random_choice":
            candidates = placement.get("candidates", [])
            if not candidates:
                raise ValueError("random_choice placement requires non-empty candidates")
            return self.rng.choice(candidates)

        raise ValueError(f"Unsupported placement mode: {mode}")

    @staticmethod
    def _vec(values):
        return " ".join(str(v) for v in values)

    @staticmethod
    def _rgba(values):
        return " ".join(str(v) for v in values)