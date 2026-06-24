#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from emdb_simulator.core.scene_loader import RoboCasaSceneLoader


class KitchenSceneInspectorNode(Node):
    def __init__(self):
        super().__init__('kitchen_scene_inspector_node')

        self.declare_parameter('layout_id', 1)
        self.declare_parameter('style_id', 1)
        self.declare_parameter('max_preview', 15)

        layout_id = self.get_parameter('layout_id').value
        style_id = self.get_parameter('style_id').value
        max_preview = self.get_parameter('max_preview').value

        loader = RoboCasaSceneLoader()
        arena = loader.create_kitchen_arena(layout_id, style_id)
        fixtures = arena.get_fixture_cfgs()

        self.get_logger().info(f'KitchenArena loaded: layout_id={layout_id}, style_id={style_id}')
        self.get_logger().info(f'num_fixtures={len(fixtures)}')

        for i, fx in enumerate(fixtures[:max_preview]):
            name = fx.get('name', 'unknown')
            typ = fx.get('type', 'unknown')
            model = fx.get('model', None)
            model_cls = type(model).__name__ if model is not None else 'None'
            self.get_logger().info(f'fixture[{i}] name={name} type={typ} model={model_cls}')


def main(args=None):
    rclpy.init(args=args)
    node = KitchenSceneInspectorNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()