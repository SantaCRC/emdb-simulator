from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from emdb_simulator.core.base_arena_loader import BaseArenaLoader
from emdb_simulator.core.world_layout_loader import WorldLayoutLoader


class TestWorldLoaderNode(Node):
    def __init__(self):
        super().__init__("test_world_loader")

        self.declare_parameter("base_arena_file", "config/arenas/base_arena.yaml")
        self.declare_parameter("world_file", "worlds/kitchen_test.yaml")
        self.declare_parameter("rate_hz", 60.0)

        base_arena_file = self.get_parameter("base_arena_file").get_parameter_value().string_value
        world_file = self.get_parameter("world_file").get_parameter_value().string_value
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value

        share_dir = Path(get_package_share_directory("emdb_simulator"))
        base_arena_path = share_dir / base_arena_file
        world_yaml_path = share_dir / "config" / world_file
        generated_xml_path = Path("/tmp/emdb_world.xml")

        self.get_logger().info(f"Base arena: {base_arena_path}")
        self.get_logger().info(f"World YAML: {world_yaml_path}")

        base_loader = BaseArenaLoader(str(base_arena_path))
        root, asset, worldbody = base_loader.load()

        world_loader = WorldLayoutLoader(str(world_yaml_path))
        world_loader.apply(root, asset, worldbody)

        ET.ElementTree(root).write(generated_xml_path, encoding="utf-8", xml_declaration=True)

        self.get_logger().info(f"Generated XML: {generated_xml_path}")

        self.model = mujoco.MjModel.from_xml_path(str(generated_xml_path))
        self.data = mujoco.MjData(self.model)

        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )

        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.step_simulation)

        self.get_logger().info(f"Simulator started at {rate_hz:.1f} Hz")

    def step_simulation(self):
        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

        self.get_logger().info("Shutting down simulator node")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TestWorldLoaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()