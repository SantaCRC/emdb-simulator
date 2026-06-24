import os
import copy
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
        self.config_dir = os.path.dirname(self.objects_yaml_path)
        self.package_root = os.path.dirname(self.config_dir)

    def apply(self, root: ET.Element, asset: ET.Element, worldbody: ET.Element):
        for obj_cfg in self.config.get("objects", []):
            self._add_object(root, asset, worldbody, obj_cfg)

    def _add_object(self, root: ET.Element, asset: ET.Element, worldbody: ET.Element, cfg: dict):
        obj_type = cfg["type"].lower()

        if obj_type in {"box", "sphere", "cylinder", "capsule", "ellipsoid"}:
            self._add_primitive_object(worldbody, cfg)
        elif obj_type in {"xml", "mjcf"}:
            self._add_xml_object(root, asset, worldbody, cfg)
        else:
            raise ValueError(f"Unsupported object type: {cfg['type']}")

    def _add_primitive_object(self, worldbody: ET.Element, cfg: dict):
        pos = self._resolve_position(cfg.get("placement", {}))

        body = ET.SubElement(
            worldbody,
            "body",
            name=cfg["name"],
            pos=self._vec(pos),
        )

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

        for key in ("density", "mass", "group", "contype", "conaffinity"):
            if key in cfg:
                geom_attrs[key] = str(cfg[key])

        for key in ("friction", "solref", "solimp"):
            if key in cfg:
                geom_attrs[key] = self._vec(cfg[key])

        ET.SubElement(body, "geom", **geom_attrs)

    def _add_xml_object(self, root: ET.Element, asset: ET.Element, worldbody: ET.Element, cfg: dict):
        pos = self._resolve_position(cfg.get("placement", {}))
        obj_name = cfg["name"]

        xml_path = cfg.get("xml_path")
        if not xml_path:
            raise ValueError(f"XML object '{obj_name}' requires 'xml_path'")

        abs_xml_path = os.path.abspath(os.path.join(self.package_root, xml_path))
        if not os.path.exists(abs_xml_path):
            raise FileNotFoundError(f"Object XML not found: {abs_xml_path}")

        xml_root = ET.parse(abs_xml_path).getroot()

        xml_default = xml_root.find("default")
        xml_asset = xml_root.find("asset")
        xml_worldbody = xml_root.find("worldbody")

        if xml_default is not None:
            root_default = root.find("default")
            if root_default is None:
                root_default = ET.SubElement(root, "default")
            self._merge_default_section(root_default, xml_default, obj_name)

        if xml_asset is not None:
            self._merge_asset_section(asset, xml_asset, os.path.dirname(abs_xml_path), obj_name)

        if xml_worldbody is None:
            raise ValueError(f"Object XML has no <worldbody>: {abs_xml_path}")

        top_bodies = xml_worldbody.findall("body")
        if not top_bodies:
            raise ValueError(f"Object XML has no <body> in <worldbody>: {abs_xml_path}")

        wrapper = ET.SubElement(
            worldbody,
            "body",
            name=obj_name,
            pos=self._vec(pos),
        )

        if cfg.get("freejoint", True):
            ET.SubElement(wrapper, "freejoint")

        for body in top_bodies:
            body_copy = copy.deepcopy(body)
            self._prefix_body_tree(body_copy, obj_name)
            wrapper.append(body_copy)

    def _merge_default_section(self, main_default: ET.Element, xml_default: ET.Element, prefix: str):
        for child in list(xml_default):
            child_copy = copy.deepcopy(child)
            self._prefix_default_tree(child_copy, prefix)
            main_default.append(child_copy)

    def _merge_asset_section(self, main_asset: ET.Element, xml_asset: ET.Element, xml_dir: str, prefix: str):
        existing = {(elem.tag, elem.attrib.get("name")) for elem in list(main_asset)}

        for elem in list(xml_asset):
            elem_copy = copy.deepcopy(elem)

            if "name" in elem_copy.attrib:
                elem_copy.attrib["name"] = f"{prefix}_{elem_copy.attrib['name']}"

            if elem_copy.tag in {"mesh", "texture", "hfield", "skin"} and "file" in elem_copy.attrib:
                rel_file = elem_copy.attrib["file"]
                abs_file = os.path.abspath(os.path.join(xml_dir, rel_file))
                elem_copy.attrib["file"] = abs_file

            if elem_copy.tag == "material" and "texture" in elem_copy.attrib:
                elem_copy.attrib["texture"] = f"{prefix}_{elem_copy.attrib['texture']}"

            key = (elem_copy.tag, elem_copy.attrib.get("name"))
            if key not in existing:
                main_asset.append(elem_copy)

    def _prefix_default_tree(self, elem: ET.Element, prefix: str):
        if "class" in elem.attrib:
            elem.attrib["class"] = f"{prefix}_{elem.attrib['class']}"
        if "childclass" in elem.attrib:
            elem.attrib["childclass"] = f"{prefix}_{elem.attrib['childclass']}"

        for ref in ("mesh", "material", "texture"):
            if ref in elem.attrib:
                elem.attrib[ref] = f"{prefix}_{elem.attrib[ref]}"

        for child in list(elem):
            self._prefix_default_tree(child, prefix)

    def _prefix_body_tree(self, elem: ET.Element, prefix: str):
        if "name" in elem.attrib:
            elem.attrib["name"] = f"{prefix}_{elem.attrib['name']}"

        for ref in ("mesh", "material", "texture", "class", "childclass"):
            if ref in elem.attrib:
                elem.attrib[ref] = f"{prefix}_{elem.attrib[ref]}"

        for child in list(elem):
            self._prefix_body_tree(child, prefix)

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