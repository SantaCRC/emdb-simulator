"""ROS 2 bridge for controlling a RoboCasa / robosuite kitchen environment with live rendering."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterable, List


def _add_virtualenv_site_packages_to_path() -> None:
    venv = os.getenv("VIRTUAL_ENV")
    if not venv:
        return
    lib_dir = Path(venv) / "lib"
    if not lib_dir.exists():
        return
    for candidate in sorted(lib_dir.glob("python*/site-packages")):
        path_str = str(candidate)
        if candidate.exists() and path_str not in sys.path:
            sys.path.insert(0, path_str)


_add_virtualenv_site_packages_to_path()


import numpy as np
import rclpy
import robosuite
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

try:
    import robocasa  # noqa: F401
    from robosuite.controllers import load_composite_controller_config
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType
    from robocasa.wrappers.enclosing_wall_render_wrapper import (
        EnclosingWallRenderWrapper,
        install_enclosing_wall_hotkeys,
    )
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "robocasa / robosuite not found. Activate your venv and ensure ros2 run sees it."
    ) from exc


class RoboCasaBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("robocasa_bridge")

        self.declare_parameter("env_name", "Kitchen")
        self.declare_parameter("robots", "PandaOmron")
        self.declare_parameter("renderer", "mjviewer")
        self.declare_parameter("render_onscreen", True)
        self.declare_parameter("show_walls", False)
        self.declare_parameter("control_freq", 20)
        self.declare_parameter("sim_rate_hz", 20.0)
        self.declare_parameter("sim_steps_per_tick", 1)
        self.declare_parameter("layout_id", -1)
        self.declare_parameter("style_id", -1)

        self.env_name = str(self.get_parameter("env_name").value)
        self.robots = str(self.get_parameter("robots").value)
        self.renderer = str(self.get_parameter("renderer").value)
        self.render_onscreen = bool(self.get_parameter("render_onscreen").value)
        self.show_walls = bool(self.get_parameter("show_walls").value)
        self.control_freq = int(self.get_parameter("control_freq").value)
        self.sim_rate_hz = float(self.get_parameter("sim_rate_hz").value)
        self.sim_steps_per_tick = int(self.get_parameter("sim_steps_per_tick").value)
        self.layout_id = int(self.get_parameter("layout_id").value)
        self.style_id = int(self.get_parameter("style_id").value)

        self._episode_done_halted = False
        self._joint_warned = False

        config = {
            "env_name": self.env_name,
            "robots": self.robots,
            "controller_configs": load_composite_controller_config(robot=self.robots),
            "translucent_robot": False,
        }

        self.env = robosuite.make(
            **config,
            has_renderer=self.render_onscreen,
            has_offscreen_renderer=False,
            render_camera=None,
            ignore_done=True,
            use_camera_obs=False,
            control_freq=self.control_freq,
            renderer=self.renderer,
        )

        self.env = EnclosingWallRenderWrapper(
            self.env,
            alpha=0.1,
            enabled=not self.show_walls,
        )
        install_enclosing_wall_hotkeys(self.env)

        valid_layout_ids = [int(x.value) for x in LayoutType if int(x.value) >= 0]
        valid_style_ids = [int(x.value) for x in StyleType if int(x.value) >= 0]

        chosen_layout = self.layout_id if self.layout_id in valid_layout_ids else int(np.random.choice(valid_layout_ids))
        chosen_style = self.style_id if self.style_id in valid_style_ids else int(np.random.choice(valid_style_ids))

        self.env.layout_and_style_ids = [[chosen_layout, chosen_style]]
        self.get_logger().info(f"Using layout_id={chosen_layout}, style_id={chosen_style}")

        self._last_obs = self.env.reset()

        low, high = self.env.action_spec
        self.action_low = np.asarray(low, dtype=float).reshape(-1)
        self.action_high = np.asarray(high, dtype=float).reshape(-1)
        self.action_dim = int(self.action_low.shape[0])
        self.current_action = np.zeros(self.action_dim, dtype=float)

        action_spec_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.joint_state_pub = self.create_publisher(JointState, "state/joint_states", 10)
        self.action_spec_pub = self.create_publisher(
            Float64MultiArray, "state/action_spec", action_spec_qos
        )

        self.create_subscription(Float64MultiArray, "cmd/action", self._action_cb, 10)
        self.create_service(Trigger, "reset", self._reset_cb)
        self.create_service(Trigger, "reset_episode", self._reset_cb)
        self.create_service(Trigger, "zero_action", self._zero_action_cb)

        self._publish_action_spec()
        self.action_spec_timer = self.create_timer(1.0, self._publish_action_spec)

        dt = 1.0 / max(1e-6, self.sim_rate_hz)
        self.timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f"robocasa_bridge started: env_name={self.env_name} robots={self.robots} "
            f"renderer={self.renderer} render_onscreen={self.render_onscreen} "
            f"action_dim={self.action_dim}"
        )

    def _clip_action(self) -> None:
        np.clip(self.current_action, self.action_low, self.action_high, out=self.current_action)

    def _publish_action_spec(self) -> None:
        payload: List[float] = [float(self.action_dim)]
        payload.extend(self.action_low.tolist())
        payload.extend(self.action_high.tolist())
        msg = Float64MultiArray()
        msg.data = payload
        self.action_spec_pub.publish(msg)

    def _action_cb(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=float)
        if values.shape[0] != self.action_dim:
            self.get_logger().warn(
                f"cmd/action expects {self.action_dim} values, got {values.shape[0]}"
            )
            return
        self.current_action[:] = values
        self._clip_action()

    @staticmethod
    def _safe_joint_names(names: Iterable[str], n_joints: int) -> List[str]:
        out = [str(n) for n in names]
        if len(out) >= n_joints:
            return out[:n_joints]
        return out + [f"joint_{i}" for i in range(len(out), n_joints)]

    def _get_robot_ref(self):
        try:
            if hasattr(self.env, "robots") and len(self.env.robots) > 0:
                return self.env.robots[0]
        except Exception:
            pass
        try:
            if hasattr(self.env, "env") and hasattr(self.env.env, "robots") and len(self.env.env.robots) > 0:
                return self.env.env.robots[0]
        except Exception:
            pass
        return None

    def _extract_joint_state(self) -> tuple[List[str], np.ndarray]:
        robot = self._get_robot_ref()

        if robot is not None and hasattr(robot, "get_qpos"):
            try:
                qpos = np.asarray(robot.get_qpos(), dtype=float).reshape(-1)
                if hasattr(robot, "robot_model") and hasattr(robot.robot_model, "joints"):
                    names = [joint.name for joint in robot.robot_model.joints]
                elif hasattr(robot, "joint_names"):
                    names = [str(n) for n in robot.joint_names]
                else:
                    names = []
                if qpos.size > 0:
                    return self._safe_joint_names(names, qpos.shape[0]), qpos
            except Exception:
                pass

        try:
            sim = None
            if hasattr(self.env, "sim"):
                sim = self.env.sim
            elif hasattr(self.env, "env") and hasattr(self.env.env, "sim"):
                sim = self.env.env.sim
            if sim is not None and hasattr(sim, "data") and hasattr(sim.data, "qpos"):
                qpos = np.asarray(sim.data.qpos, dtype=float).reshape(-1)
                if qpos.size > 0:
                    return self._safe_joint_names([], qpos.shape[0]), qpos
        except Exception:
            pass

        obs = self._last_obs
        if isinstance(obs, dict):
            try:
                if isinstance(obs.get("agent"), dict) and "qpos" in obs["agent"]:
                    qpos = np.asarray(obs["agent"]["qpos"], dtype=float).reshape(-1)
                    return self._safe_joint_names([], qpos.shape[0]), qpos
                if "qpos" in obs:
                    qpos = np.asarray(obs["qpos"], dtype=float).reshape(-1)
                    return self._safe_joint_names([], qpos.shape[0]), qpos
                if "robot0_joint_pos" in obs:
                    qpos = np.asarray(obs["robot0_joint_pos"], dtype=float).reshape(-1)
                    return self._safe_joint_names([], qpos.shape[0]), qpos
                if "robot0_gripper_qpos" in obs:
                    qpos = np.asarray(obs["robot0_gripper_qpos"], dtype=float).reshape(-1)
                    return self._safe_joint_names([], qpos.shape[0]), qpos
            except Exception:
                pass

        return [], np.asarray([], dtype=float)

    def _publish_joint_state(self) -> None:
        names, qpos = self._extract_joint_state()
        if qpos.shape[0] == 0:
            if not self._joint_warned:
                self.get_logger().warn("Could not extract joint state from RoboCasa env")
                self._joint_warned = True
            return
        self._joint_warned = False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = qpos.astype(float).tolist()
        self.joint_state_pub.publish(msg)

    def _tick(self) -> None:
        if self._episode_done_halted:
            if self.render_onscreen:
                self.env.render()
            self._publish_joint_state()
            return

        for _ in range(max(1, self.sim_steps_per_tick)):
            step_out = self.env.step(self.current_action)

            if isinstance(step_out, tuple) and len(step_out) >= 1:
                self._last_obs = step_out[0]

            if self.render_onscreen:
                self.env.render()

        self._publish_joint_state()

    def _reset_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._last_obs = self.env.reset()
        self.current_action[:] = 0.0
        self._clip_action()
        self._episode_done_halted = False
        self._publish_joint_state()
        response.success = True
        response.message = "Environment reset and action command zeroed"
        return response

    def _zero_action_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.current_action[:] = 0.0
        self._clip_action()
        response.success = True
        response.message = "Action command set to zero"
        return response

    def destroy_node(self) -> bool:
        try:
            self.env.close()
        except Exception as exc:
            self.get_logger().warn(f"Failed to close RoboCasa env cleanly: {exc}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: RoboCasaBridgeNode | None = None
    try:
        node = RoboCasaBridgeNode()
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