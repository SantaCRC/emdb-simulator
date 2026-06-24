from pathlib import Path
import random
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from emdb_simulator.core.base_arena_loader import BaseArenaLoader
from emdb_simulator.core.world_layout_loader import WorldLayoutLoader
from emdb_simulator.core.object_loader import ObjectLoader


class TestObjectLoaderNode(Node):
    def __init__(self):
        super().__init__("test_object_loader")

        self.declare_parameter("base_arena_file", "config/arenas/base_arena.yaml")
        self.declare_parameter("world_file", "worlds/kitchen_test.yaml")
        self.declare_parameter("objects_file", "objects/objects_test.yaml")
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("generated_xml_path", "/tmp/emdb_objects_test.xml")

        self.base_arena_file = self.get_parameter(
            "base_arena_file"
        ).get_parameter_value().string_value
        self.world_file = self.get_parameter(
            "world_file"
        ).get_parameter_value().string_value
        self.objects_file = self.get_parameter(
            "objects_file"
        ).get_parameter_value().string_value
        self.rate_hz = self.get_parameter(
            "rate_hz"
        ).get_parameter_value().double_value
        self.generated_xml_path = Path(
            self.get_parameter("generated_xml_path").get_parameter_value().string_value
        )

        self.share_dir = Path(get_package_share_directory("emdb_simulator"))
        self.base_arena_path = self.share_dir / self.base_arena_file
        self.world_yaml_path = self.share_dir / "config" / self.world_file
        self.objects_yaml_path = self.share_dir / "config" / self.objects_file

        self.model = None
        self.data = None
        self.viewer = None
        self.timer = None
        self.episode_idx = 0

        self.get_logger().info(f"Base arena: {self.base_arena_path}")
        self.get_logger().info(f"World file: {self.world_yaml_path}")
        self.get_logger().info(f"Objects file: {self.objects_yaml_path}")
        self.get_logger().info(f"Generated XML: {self.generated_xml_path}")

        self.reset_srv = self.create_service(
            Trigger,
            "/simulator/reset_world",
            self.handle_reset_world,
        )

        self.reset_episode()

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.step_simulation)

        self.get_logger().info(
            f"Simulator started. rate_hz={self.rate_hz:.1f}, service=/simulator/reset_world"
        )

    def reset_episode(self):
        self.episode_idx += 1
        self.get_logger().info(f"Resetting episode {self.episode_idx}")

        root, asset, worldbody = BaseArenaLoader(str(self.base_arena_path)).load()
        WorldLayoutLoader(str(self.world_yaml_path)).apply(root, asset, worldbody)

        rng = random.Random()
        ObjectLoader(str(self.objects_yaml_path), rng=rng).apply(root, asset, worldbody)

        self._log_worldbody_objects(worldbody)

        self.generated_xml_path.parent.mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(
            self.generated_xml_path,
            encoding="utf-8",
            xml_declaration=True,
        )

        old_viewer = self.viewer
        self.viewer = None

        self.model = mujoco.MjModel.from_xml_path(str(self.generated_xml_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)

        if old_viewer is not None:
            try:
                old_viewer.close()
            except Exception:
                pass

        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self.viewer.sync()

        self.get_logger().info(
            f"Episode {self.episode_idx} ready. XML regenerated at {self.generated_xml_path}"
        )

    def handle_reset_world(self, request, response):
        try:
            self.reset_episode()
            response.success = True
            response.message = f"World reset completed. episode={self.episode_idx}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"World reset failed: {e}"
            self.get_logger().error(response.message)

        return response

    def step_simulation(self):
        if self.model is None or self.data is None:
            return

        mujoco.mj_step(self.model, self.data)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def _log_worldbody_objects(self, worldbody: ET.Element):
        bodies = worldbody.findall("body")
        self.get_logger().info(f"Worldbody contains {len(bodies)} top-level bodies")

        for body in bodies:
            name = body.attrib.get("name", "<unnamed>")
            pos = body.attrib.get("pos", "0 0 0")
            geoms = body.findall("geom")
            if geoms:
                geom_types = [g.attrib.get("type", "?") for g in geoms]
                self.get_logger().info(
                    f"Body loaded: name={name}, pos={pos}, geoms={geom_types}"
                )

    def destroy_node(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = TestObjectLoaderNode()
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