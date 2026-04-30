"""ROS 2 bridge for controlling a ManiSkill simulation.

The node keeps a running ManiSkill environment and exposes command streaming
over topics plus utility services for reset and zeroing commands.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Iterable, List


def _add_virtualenv_site_packages_to_path() -> None:
    """Best-effort helper so ros2-run system Python can see venv packages."""

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


# Ensure venv packages take priority over system Python.
_add_virtualenv_site_packages_to_path()


import numpy as np
import rclpy
from gymnasium import make as gym_make
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

try:
    import mani_skill  # noqa: F401
except ModuleNotFoundError as exc:
    _add_virtualenv_site_packages_to_path()
    try:
        import mani_skill  # noqa: F401
    except ModuleNotFoundError:
        raise ModuleNotFoundError(
            "mani_skill not found. Activate your venv and ensure its site-packages are visible "
            "to ros2 run, e.g. export PYTHONPATH=$VIRTUAL_ENV/lib/python3.10/site-packages:$PYTHONPATH"
        ) from exc

# Registers custom robot uid="ur5e" before env creation.
import maniskill_ros.robots.ur5e  # noqa: F401


def _register_robocasa_front_facing(robot_uid: str, front_facing_size: float) -> None:
    """Best-effort patch for RoboCasa robot front-facing size table."""

    try:
        from mani_skill.utils.scene_builder.robocasa.scene_builder import (  # type: ignore
            ROBOT_FRONT_FACING_SIZE,
        )
    except Exception:
        return

    ROBOT_FRONT_FACING_SIZE[str(robot_uid)] = float(front_facing_size)


class ManiSkillBridgeNode(Node):
    """ROS 2 node that bridges ManiSkill controls with ROS topics/services."""

    def __init__(self) -> None:
        super().__init__("maniskill_bridge")

        self.declare_parameter("env_id", "RoboCasaKitchen-v1")
        self.declare_parameter("robot_uid", "ur5e")
        self.declare_parameter("obs_mode", "state")
        self.declare_parameter("control_mode", "pd_joint_pos")
        self.declare_parameter("render_mode", "human")
        self.declare_parameter("sim_rate_hz", 60.0)
        self.declare_parameter("sim_steps_per_tick", 1)
        self.declare_parameter("auto_reset_on_done", True)
        self.declare_parameter("reset_seed", -1)
        self.declare_parameter("seed", 2)

        self.env_id = str(self.get_parameter("env_id").value)
        self.robot_uid = str(self.get_parameter("robot_uid").value)
        self.obs_mode = str(self.get_parameter("obs_mode").value)
        self.control_mode = str(self.get_parameter("control_mode").value)
        self.render_mode = str(self.get_parameter("render_mode").value)
        self.sim_rate_hz = float(self.get_parameter("sim_rate_hz").value)
        self.sim_steps_per_tick = int(self.get_parameter("sim_steps_per_tick").value)
        self.auto_reset_on_done = bool(self.get_parameter("auto_reset_on_done").value)
        self.reset_seed = int(self.get_parameter("reset_seed").value)
        self.seed = int(self.get_parameter("seed").value)

        # Add custom robot to RoboCasa front-facing lookup to avoid fallback warning.
        _register_robocasa_front_facing(self.robot_uid, 0.5)

        render_mode = None if self.render_mode.lower() in {"", "none", "off"} else self.render_mode
        try:
            self.env = gym_make(
                self.env_id,
                obs_mode=self.obs_mode,
                reward_mode="none",
                control_mode=self.control_mode,
                robot_uids=self.robot_uid,
                render_mode=render_mode,
                seed=self.seed,

            )
        except NotImplementedError as exc:
            raise RuntimeError(
                "Failed to create ManiSkill env. The selected env_id may require robot-specific "
                "methods that this robot does not implement. For custom robots, start with env_id=Empty-v1."
            ) from exc

        reset_kwargs = {}
        if self.reset_seed >= 0:
            reset_kwargs["seed"] = self.reset_seed

        self._last_obs, _ = self.env.reset(**reset_kwargs)

        action_space = self.env.action_space
        self.action_low = np.asarray(action_space.low, dtype=float).reshape(-1)
        self.action_high = np.asarray(action_space.high, dtype=float).reshape(-1)
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
        self.create_service(Trigger, "zero_action", self._zero_action_cb)

        self._publish_action_spec()
        self.action_spec_timer = self.create_timer(1.0, self._publish_action_spec)

        dt = 1.0 / max(1e-6, self.sim_rate_hz)
        self.timer = self.create_timer(dt, self._tick)
        self._joint_warned = False
        self._episode_done_halted = False

        self.get_logger().info(
            f"maniskill_bridge started: env_id={self.env_id} robot_uid={self.robot_uid} "
            f"obs_mode={self.obs_mode} control_mode={self.control_mode} render_mode={self.render_mode} "
            f"action_dim={self.action_dim} auto_reset_on_done={self.auto_reset_on_done} "
            f"reset_seed={self.reset_seed}"
        )

        self._should_render = self.render_mode.lower() == "human"

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

    def _extract_joint_state(self) -> tuple[List[str], np.ndarray]:
        env_unwrapped = self.env.unwrapped

        agent = getattr(env_unwrapped, "agent", None)
        robot = getattr(agent, "robot", None) if agent is not None else None

        if robot is not None and hasattr(robot, "get_qpos"):
            try:
                qpos = np.asarray(robot.get_qpos(), dtype=float).reshape(-1)
                if hasattr(robot, "get_active_joints"):
                    names = [joint.name for joint in robot.get_active_joints()]
                else:
                    names = []
                return self._safe_joint_names(names, qpos.shape[0]), qpos
            except Exception:
                pass

        obs = self._last_obs
        if isinstance(obs, dict):
            if isinstance(obs.get("agent"), dict) and "qpos" in obs["agent"]:
                qpos = np.asarray(obs["agent"]["qpos"], dtype=float).reshape(-1)
                return self._safe_joint_names([], qpos.shape[0]), qpos
            if "qpos" in obs:
                qpos = np.asarray(obs["qpos"], dtype=float).reshape(-1)
                return self._safe_joint_names([], qpos.shape[0]), qpos

        return [], np.asarray([], dtype=float)

    def _publish_joint_state(self) -> None:
        names, qpos = self._extract_joint_state()
        if qpos.shape[0] == 0:
            if not self._joint_warned:
                self.get_logger().warn("Could not extract joint state from ManiSkill env")
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
            return

        for _ in range(max(1, self.sim_steps_per_tick)):
            step_out = self.env.step(self.current_action)

            terminated = False
            truncated = False
            if isinstance(step_out, tuple):
                if len(step_out) >= 1:
                    self._last_obs = step_out[0]
                if len(step_out) >= 3:
                    terminated = bool(step_out[2])
                if len(step_out) >= 4:
                    truncated = bool(step_out[3])

            if terminated or truncated:
                if self.auto_reset_on_done:
                    self.get_logger().info("Episode finished, resetting environment")
                    self._last_obs, _ = self.env.reset()
                    self.current_action[:] = 0.0
                    self._clip_action()
                    self._episode_done_halted = False
                    break

                self.get_logger().warn(
                    "Episode finished but auto_reset_on_done=false; stopping until reset"
                )
                self._episode_done_halted = True
                return

            if self._should_render:
                self.env.render()

        self._publish_joint_state()

    def _reset_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        reset_kwargs = {}
        if self.reset_seed >= 0:
            reset_kwargs["seed"] = self.reset_seed

        self._last_obs, _ = self.env.reset(**reset_kwargs)
        self.current_action[:] = 0.0
        self._clip_action()
        self._episode_done_halted = False
        response.success = True
        response.message = "Environment reset and action command zeroed"
        return response

    def _zero_action_cb(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.current_action[:] = 0.0
        self._clip_action()
        response.success = True
        response.message = "Action command set to zero"
        return response

    def destroy_node(self) -> bool:
        try:
            self.env.close()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Failed to close ManiSkill env cleanly: {exc}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ManiSkillBridgeNode | None = None
    try:
        node = ManiSkillBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow Ctrl+C without surfacing shutdown stack traces.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
