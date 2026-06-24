from pathlib import Path
import xml.etree.ElementTree as ET


class BaseArenaLoader:
    def __init__(self, base_arena_xml_path: str):
        self.base_arena_xml_path = Path(base_arena_xml_path).resolve()

        if not self.base_arena_xml_path.exists():
            raise FileNotFoundError(
                f"Base arena XML not found: {self.base_arena_xml_path}"
            )

    def load_tree(self) -> ET.ElementTree:
        return ET.parse(self.base_arena_xml_path)

    def load_root(self) -> ET.Element:
        tree = self.load_tree()
        root = tree.getroot()

        if root.tag != "mujoco":
            raise ValueError(
                f"Invalid MJCF root tag '{root.tag}', expected 'mujoco'"
            )

        self._ensure_child(root, "asset")
        self._ensure_child(root, "worldbody")

        return root

    def load(self):
        root = self.load_root()
        asset = root.find("asset")
        worldbody = root.find("worldbody")
        return root, asset, worldbody

    @staticmethod
    def _ensure_child(root: ET.Element, tag: str) -> ET.Element:
        child = root.find(tag)
        if child is None:
            child = ET.SubElement(root, tag)
        return child