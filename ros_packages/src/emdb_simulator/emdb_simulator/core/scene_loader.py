from dataclasses import dataclass
from typing import Dict, Tuple, Union

from robocasa.models.scenes.kitchen_arena import KitchenArena
from robocasa.models.scenes.scene_registry import LayoutType, StyleType


LayoutLike = Union[int, LayoutType]
StyleLike = Union[int, StyleType]


@dataclass
class SceneRegistryInfo:
    layouts: Dict[int, str]
    styles: Dict[int, str]


class RoboCasaSceneLoader:
    def list_available_layouts(self, include_special: bool = False) -> Dict[int, str]:
        items = {}
        for item in LayoutType:
            value = int(item.value)
            if include_special or value >= 0:
                items[value] = item.name
        return items

    def list_available_styles(self, include_special: bool = False) -> Dict[int, str]:
        items = {}
        for item in StyleType:
            value = int(item.value)
            if include_special or value >= 0:
                items[value] = item.name
        return items

    def get_registry_info(self) -> SceneRegistryInfo:
        return SceneRegistryInfo(
            layouts=self.list_available_layouts(),
            styles=self.list_available_styles(),
        )

    def get_default_layout_id(self) -> int:
        valid = sorted(self.list_available_layouts().keys())
        if not valid:
            raise RuntimeError("No valid non-negative LayoutType values found")
        return valid[0]

    def get_default_style_id(self) -> int:
        valid = sorted(self.list_available_styles().keys())
        if not valid:
            raise RuntimeError("No valid non-negative StyleType values found")
        return valid[0]

    def validate_ids(self, layout_id: LayoutLike, style_id: StyleLike) -> Tuple[bool, str]:
        layout_id = int(layout_id)
        style_id = int(style_id)

        layouts = self.list_available_layouts()
        styles = self.list_available_styles()

        if layout_id not in layouts:
            return False, f"Invalid layout_id={layout_id}. Available: {sorted(layouts.keys())}"
        if style_id not in styles:
            return False, f"Invalid style_id={style_id}. Available: {sorted(styles.keys())}"

        return True, "ok"

    def create_kitchen_arena(self, layout_id: LayoutLike, style_id: StyleLike) -> KitchenArena:
        layout_id = int(layout_id)
        style_id = int(style_id)

        ok, msg = self.validate_ids(layout_id, style_id)
        if not ok:
            raise ValueError(msg)

        return KitchenArena(layout_id=layout_id, style_id=style_id)