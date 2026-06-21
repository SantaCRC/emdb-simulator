import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_path

from emdb_simulator.core.base_arena_loader import BaseArenaLoader


class TestBaseArenaLoaderNode(Node):
    def __init__(self):
        super().__init__("test_base_arena_loader")

        self.declare_parameter("arena_file", "mjcf/arenas/empty_arena.xml")

        arena_file = self.get_parameter("arena_file").get_parameter_value().string_value
        share_dir = get_package_share_path("emdb_simulator")
        base_xml_path = share_dir / arena_file

        self.get_logger().info(f"Arena file parameter: {arena_file}")
        self.get_logger().info(f"Resolved arena XML path: {base_xml_path}")

        loader = BaseArenaLoader(str(base_xml_path))
        root, asset, worldbody = loader.load()

        self.get_logger().info(f"Root tag: {root.tag}")
        self.get_logger().info(f"Model name: {root.get('model')}")
        self.get_logger().info(f"Asset found: {asset is not None}")
        self.get_logger().info(f"Worldbody found: {worldbody is not None}")

        if root.tag != "mujoco":
            raise RuntimeError("Invalid MJCF root tag")
        if asset is None:
            raise RuntimeError("Missing <asset> section in base arena")
        if worldbody is None:
            raise RuntimeError("Missing <worldbody> section in base arena")

        self.get_logger().info("BaseArenaLoader test passed")


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TestBaseArenaLoaderNode()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()