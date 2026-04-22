"""ROS 2 bridge for controlling a robosuite UR5e + QbHand2M simulation.

The node keeps a running robosuite environment and exposes efficient command
streaming over topics, plus utility services for reset and zeroing commands.
"""

from __future__ import annotations

import asyncio
import os
import sys
from pathlib import Path
from typing import List


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


# Ensure venv packages (numpy/numba/robosuite) take priority over system Python.
_add_virtualenv_site_packages_to_path()

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

try:
    import robosuite as suite
except ModuleNotFoundError as exc:
    _add_virtualenv_site_packages_to_path()
    try:
        import robosuite as suite
    except ModuleNotFoundError:
        raise ModuleNotFoundError(
            "robosuite not found. Activate your venv and ensure its site-packages are visible "
            "to ros2 run, e.g. export PYTHONPATH=$VIRTUAL_ENV/lib/python3.10/site-packages:$PYTHONPATH"
        ) from exc

from robosuite_ros.grippers.qbhand2m_gripper import QbHand2MGripper  # noqa: F401


class RobosuiteBridgeNode(Node):
    """ROS 2 node that bridges robosuite controls with ROS topics/services."""

    def __init__(self) -> None:
        super().__init__("robosuite_bridge")

        self.declare_parameter("env_name", "Lift")
        self.declare_parameter("robot", "UR5e")
        self.declare_parameter("gripper", "QbHand2MGripper")
        self.declare_parameter("control_rate_hz", 60.0)
        self.declare_parameter("sim_steps_per_tick", 1)
        self.declare_parameter("render", os.getenv("ROBOSUITE_RENDER", "1") == "1")
        self.declare_parameter("ignore_done", True)
        self.declare_parameter("horizon", 1000000)
        self.declare_parameter("auto_reset_on_done", False)
        self.declare_parameter("arm_position_kp", 2.0)
        self.declare_parameter("arm_position_kd", 0.2)
        self.declare_parameter("arm_position_deadband", 0.005)
        self.declare_parameter("arm_action_max_abs", 0.25)
        self.declare_parameter("arm_action_slew_rate", 1.0)
        self.declare_parameter("arm_target_max_delta", 0.0)
        self.declare_parameter("arm_target_reached_tol", 0.03)
        self.declare_parameter("arm_target_reached_vel_tol", 0.05)
        self.declare_parameter("arm_target_timeout", 12.0)
        self.declare_parameter("arm_cmd_timeout", 0.5)
        self.declare_parameter("trajectory_max_duration", 20.0)

        env_name = str(self.get_parameter("env_name").value)
        robot = str(self.get_parameter("robot").value)
        gripper = str(self.get_parameter("gripper").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.sim_steps_per_tick = int(self.get_parameter("sim_steps_per_tick").value)
        self.render = bool(self.get_parameter("render").value)
        self.ignore_done = bool(self.get_parameter("ignore_done").value)
        self.horizon = int(self.get_parameter("horizon").value)
        self.auto_reset_on_done = bool(self.get_parameter("auto_reset_on_done").value)
        self.arm_position_kp = float(self.get_parameter("arm_position_kp").value)
        self.arm_position_kd = float(self.get_parameter("arm_position_kd").value)
        self.arm_position_deadband = float(self.get_parameter("arm_position_deadband").value)
        self.arm_action_max_abs = float(self.get_parameter("arm_action_max_abs").value)
        self.arm_action_slew_rate = float(self.get_parameter("arm_action_slew_rate").value)
        self.arm_target_max_delta = float(self.get_parameter("arm_target_max_delta").value)
        self.arm_target_reached_tol = float(self.get_parameter("arm_target_reached_tol").value)
        self.arm_target_reached_vel_tol = float(self.get_parameter("arm_target_reached_vel_tol").value)
        self.arm_target_timeout = float(self.get_parameter("arm_target_timeout").value)
        self.arm_cmd_timeout = float(self.get_parameter("arm_cmd_timeout").value)
        self.trajectory_max_duration = float(self.get_parameter("trajectory_max_duration").value)

        self.env = suite.make(
            env_name=env_name,
            robots=robot,
            has_renderer=self.render,
            has_offscreen_renderer=False,
            use_camera_obs=False,
            gripper_types=gripper,
            ignore_done=self.ignore_done,
            horizon=self.horizon,
        )
        self.env.reset()

        action_low, action_high = self.env.action_spec
        self.action_low = np.asarray(action_low, dtype=float)
        self.action_high = np.asarray(action_high, dtype=float)
        self.action_dim = int(self.action_low.shape[0])
        self.gripper_dim = 2
        self.arm_dim = max(0, self.action_dim - self.gripper_dim)
        self.current_action = np.zeros(self.action_dim, dtype=float)
        self.arm_action_target = np.zeros(self.arm_dim, dtype=float)
        self.arm_action_cmd = np.zeros(self.arm_dim, dtype=float)
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        self.arm_cmd_last_update = self.get_clock().now()
        self._gripper_cmd_count = 0
        self._trajectory_points = None
        self._trajectory_start_time = None
        self._trajectory_goal_handle = None
        self._trajectory_desired_vel = np.zeros(self.arm_dim, dtype=float)

        self._arm_qpos_indexes = self._resolve_arm_qpos_indexes()
        self._arm_qvel_indexes = self._resolve_arm_qvel_indexes()
        self._arm_joint_names = self._resolve_arm_joint_names()
        self._arm_joint_lower, self._arm_joint_upper = self._resolve_arm_joint_limits()

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
        self.create_subscription(Float64MultiArray, "cmd/arm", self._arm_cb, 10)
        self.create_subscription(Float64MultiArray, "cmd/arm_joint_pos", self._arm_joint_pos_cb, 10)
        self.create_subscription(JointState, "cmd/arm_joint_state", self._arm_joint_state_cb, 10)
        self.create_subscription(JointTrajectory, "cmd/arm_joint_trajectory", self._arm_joint_trajectory_cb, 10)
        self.create_subscription(Float64MultiArray, "cmd/gripper", self._gripper_cb, 10)

        self.follow_joint_trajectory_server = ActionServer(
            self,
            FollowJointTrajectory,
            "follow_joint_trajectory",
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._goal_follow_joint_trajectory,
            cancel_callback=self._cancel_follow_joint_trajectory,
        )

        self.create_service(Trigger, "reset", self._reset_cb)
        self.create_service(Trigger, "zero_action", self._zero_action_cb)

        self._publish_action_spec()
        self.action_spec_timer = self.create_timer(1.0, self._publish_action_spec)
        dt = 1.0 / max(1e-6, self.control_rate_hz)
        self.timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            f"robosuite_bridge started: env={env_name} robot={robot} "
            f"gripper={gripper} action_dim={self.action_dim} "
            f"ignore_done={self.ignore_done} horizon={self.horizon} "
            f"auto_reset_on_done={self.auto_reset_on_done} "
            f"arm_position_kp={self.arm_position_kp} "
            f"arm_position_kd={self.arm_position_kd} "
            f"arm_action_max_abs={self.arm_action_max_abs} "
            f"arm_action_slew_rate={self.arm_action_slew_rate} "
            f"arm_target_max_delta={self.arm_target_max_delta} "
            f"arm_target_timeout={self.arm_target_timeout} "
            f"trajectory_max_duration={self.trajectory_max_duration}"
        )

    def _reset_trajectory(self) -> None:
        self._trajectory_points = None
        self._trajectory_start_time = None
        self._trajectory_goal_handle = None
        self._trajectory_desired_vel[:] = 0.0

    def _parse_trajectory(
        self, trajectory: JointTrajectory
    ) -> tuple[bool, str, list[tuple[float, np.ndarray, np.ndarray]]]:
        if self.arm_dim == 0:
            return False, "arm_dim is 0", []
        if len(trajectory.points) == 0:
            return False, "trajectory has no points", []

        if len(trajectory.joint_names) == 0:
            ordered_indices = list(range(self.arm_dim))
        else:
            name_to_idx = {name: idx for idx, name in enumerate(trajectory.joint_names)}
            missing = [name for name in self._arm_joint_names if name not in name_to_idx]
            if missing:
                return False, f"trajectory missing joints {missing}", []
            ordered_indices = [name_to_idx[name] for name in self._arm_joint_names]

        parsed = []
        last_t = -1.0
        for point in trajectory.points:
            t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            if t < 0.0:
                return False, "trajectory has negative time_from_start", []
            if t < last_t:
                return False, "trajectory time_from_start must be nondecreasing", []
            last_t = t

            if len(point.positions) == 0:
                return False, "trajectory point without positions", []
            if len(point.positions) < len(ordered_indices):
                return False, "trajectory point has insufficient positions", []

            pos = np.asarray([point.positions[i] for i in ordered_indices], dtype=float)
            pos = self._sanitize_arm_joint_target(pos)

            if len(point.velocities) >= len(ordered_indices):
                vel = np.asarray([point.velocities[i] for i in ordered_indices], dtype=float)
            else:
                vel = np.zeros(self.arm_dim, dtype=float)

            parsed.append((t, pos, vel))

        if parsed[-1][0] <= 0.0:
            return False, "trajectory final time must be > 0", []
        if parsed[-1][0] > self.trajectory_max_duration:
            return (
                False,
                f"trajectory duration {parsed[-1][0]:.3f}s exceeds max {self.trajectory_max_duration:.3f}s",
                [],
            )

        return True, "", parsed

    def _start_trajectory(
        self, trajectory: JointTrajectory, goal_handle=None
    ) -> tuple[bool, str]:
        ok, reason, parsed = self._parse_trajectory(trajectory)
        if not ok:
            return False, reason

        self._trajectory_points = parsed
        self._trajectory_start_time = self.get_clock().now()
        self._trajectory_goal_handle = goal_handle
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        return True, ""

    def _trajectory_target_at(self, elapsed_s: float) -> tuple[np.ndarray, np.ndarray, bool]:
        points = self._trajectory_points
        if points is None or len(points) == 0:
            return (
                self._get_arm_joint_positions(),
                np.zeros(self.arm_dim, dtype=float),
                True,
            )

        if elapsed_s <= points[0][0]:
            return points[0][1].copy(), points[0][2].copy(), False

        for i in range(1, len(points)):
            t0, p0, v0 = points[i - 1]
            t1, p1, v1 = points[i]
            if elapsed_s <= t1:
                dt = max(1e-6, t1 - t0)
                alpha = np.clip((elapsed_s - t0) / dt, 0.0, 1.0)
                pos = p0 + alpha * (p1 - p0)
                if np.linalg.norm(v0) > 0.0 or np.linalg.norm(v1) > 0.0:
                    vel = v0 + alpha * (v1 - v0)
                else:
                    vel = (p1 - p0) / dt
                return pos, vel, False

        _, p_last, v_last = points[-1]
        return p_last.copy(), v_last.copy(), True

    def _arm_joint_trajectory_cb(self, msg: JointTrajectory) -> None:
        ok, reason = self._start_trajectory(msg)
        if not ok:
            self.get_logger().warn(f"Rejected cmd/arm_joint_trajectory: {reason}")
            return
        self.get_logger().info("Accepted cmd/arm_joint_trajectory")

    def _goal_follow_joint_trajectory(self, goal_request: FollowJointTrajectory.Goal) -> GoalResponse:
        ok, reason, _ = self._parse_trajectory(goal_request.trajectory)
        if not ok:
            self.get_logger().warn(f"Rejecting FollowJointTrajectory goal: {reason}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_follow_joint_trajectory(self, _goal_handle) -> CancelResponse:
        self._reset_trajectory()
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        self._set_arm_action_target(np.zeros(self.arm_dim, dtype=float))
        return CancelResponse.ACCEPT

    async def _execute_follow_joint_trajectory(self, goal_handle):
        ok, reason = self._start_trajectory(goal_handle.request.trajectory, goal_handle=goal_handle)
        if not ok:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = reason
            goal_handle.abort()
            return result

        while rclpy.ok() and self._trajectory_goal_handle is goal_handle:
            if goal_handle.is_cancel_requested:
                self._reset_trajectory()
                self.arm_joint_target = None
                self.arm_joint_target_set_time = None
                self._set_arm_action_target(np.zeros(self.arm_dim, dtype=float))
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Goal canceled"
                return result

            await asyncio.sleep(0.02)

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory executed"
        goal_handle.succeed()
        return result

    def _resolve_arm_qpos_indexes(self) -> np.ndarray:
        if self.arm_dim == 0:
            return np.asarray([], dtype=int)

        robot = self.env.robots[0]
        ref_indexes = getattr(robot, "_ref_joint_pos_indexes", None)
        if ref_indexes is not None:
            idx = np.asarray(ref_indexes, dtype=int).reshape(-1)
            if idx.shape[0] >= self.arm_dim:
                return idx[: self.arm_dim]

        names = list(getattr(robot, "robot_joints", []))
        idx_from_names = []
        for name in names[: self.arm_dim]:
            try:
                qpos_addr = self.env.sim.model.get_joint_qpos_addr(name)
            except Exception:
                continue
            if isinstance(qpos_addr, tuple):
                idx_from_names.append(int(qpos_addr[0]))
            else:
                idx_from_names.append(int(qpos_addr))

        if len(idx_from_names) >= self.arm_dim:
            return np.asarray(idx_from_names[: self.arm_dim], dtype=int)

        self.get_logger().warn(
            "Could not resolve all arm qpos indexes; falling back to first arm_dim qpos values"
        )
        return np.arange(self.arm_dim, dtype=int)

    def _resolve_arm_joint_names(self) -> List[str]:
        robot = self.env.robots[0]
        names = list(getattr(robot, "robot_joints", []))
        if len(names) >= self.arm_dim:
            return names[: self.arm_dim]
        return [f"arm_joint_{i}" for i in range(self.arm_dim)]

    def _resolve_arm_qvel_indexes(self) -> np.ndarray:
        if self.arm_dim == 0:
            return np.asarray([], dtype=int)

        robot = self.env.robots[0]
        ref_indexes = getattr(robot, "_ref_joint_vel_indexes", None)
        if ref_indexes is not None:
            idx = np.asarray(ref_indexes, dtype=int).reshape(-1)
            if idx.shape[0] >= self.arm_dim:
                return idx[: self.arm_dim]

        sim_model = self.env.sim.model
        idx_from_names = []
        for name in self._arm_joint_names:
            try:
                jnt_id = int(sim_model.joint_name2id(name))
                dof_adr = int(sim_model.jnt_dofadr[jnt_id])
            except Exception:
                continue
            idx_from_names.append(dof_adr)

        if len(idx_from_names) >= self.arm_dim:
            return np.asarray(idx_from_names[: self.arm_dim], dtype=int)

        self.get_logger().warn(
            "Could not resolve all arm qvel indexes; falling back to first arm_dim qvel values"
        )
        return np.arange(self.arm_dim, dtype=int)

    def _resolve_arm_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        if self.arm_dim == 0:
            empty = np.asarray([], dtype=float)
            return empty, empty

        lower = np.full(self.arm_dim, -np.inf, dtype=float)
        upper = np.full(self.arm_dim, np.inf, dtype=float)
        sim_model = self.env.sim.model

        for i, name in enumerate(self._arm_joint_names):
            try:
                jnt_id = int(sim_model.joint_name2id(name))
            except Exception:
                continue

            if int(sim_model.jnt_limited[jnt_id]) == 1:
                lower[i] = float(sim_model.jnt_range[jnt_id][0])
                upper[i] = float(sim_model.jnt_range[jnt_id][1])

        return lower, upper

    def _get_arm_joint_positions(self) -> np.ndarray:
        if self.arm_dim == 0:
            return np.asarray([], dtype=float)
        return np.asarray(self.env.sim.data.qpos[self._arm_qpos_indexes], dtype=float)

    def _get_arm_joint_velocities(self) -> np.ndarray:
        if self.arm_dim == 0:
            return np.asarray([], dtype=float)
        return np.asarray(self.env.sim.data.qvel[self._arm_qvel_indexes], dtype=float)

    def _sanitize_arm_joint_target(self, raw_target: np.ndarray) -> np.ndarray:
        target = np.asarray(raw_target[: self.arm_dim], dtype=float).copy()

        # Keep targets within physical joint limits when available.
        target = np.clip(target, self._arm_joint_lower, self._arm_joint_upper)

        # Optional: limit jump size from current joint position.
        # Set arm_target_max_delta <= 0 to disable this clamp.
        max_delta = float(self.arm_target_max_delta)
        if max_delta > 0.0:
            current = self._get_arm_joint_positions()
            target = np.clip(target, current - max_delta, current + max_delta)
        return target

    def _clip_action(self) -> None:
        np.clip(self.current_action, self.action_low, self.action_high, out=self.current_action)

    def _set_arm_action_target(self, values: np.ndarray) -> None:
        if self.arm_dim == 0:
            return
        target = np.asarray(values[: self.arm_dim], dtype=float)
        target = np.clip(target, -self.arm_action_max_abs, self.arm_action_max_abs)
        self.arm_action_target[:] = target

    def _update_arm_action_cmd(self) -> None:
        if self.arm_dim == 0:
            return

        dt = 1.0 / max(1e-6, self.control_rate_hz)
        max_delta = max(1e-6, self.arm_action_slew_rate) * dt
        delta = self.arm_action_target - self.arm_action_cmd
        delta = np.clip(delta, -max_delta, max_delta)
        self.arm_action_cmd += delta
        self.current_action[: self.arm_dim] = self.arm_action_cmd

    def _action_cb(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=float)
        if values.shape[0] != self.action_dim:
            self.get_logger().warn(
                f"cmd/action expects {self.action_dim} values, got {values.shape[0]}"
            )
            return
        self.current_action[:] = values
        if self.arm_dim > 0:
            self._set_arm_action_target(values[: self.arm_dim])
            self.arm_cmd_last_update = self.get_clock().now()
        self._reset_trajectory()
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        self._clip_action()

    def _arm_cb(self, msg: Float64MultiArray) -> None:
        if self.arm_dim == 0:
            self.get_logger().warn("cmd/arm ignored because arm_dim is 0")
            return

        values = np.asarray(msg.data, dtype=float)
        if values.shape[0] < self.arm_dim:
            self.get_logger().warn(
                f"cmd/arm expects at least {self.arm_dim} values, got {values.shape[0]}"
            )
            return
        self._set_arm_action_target(values)
        self.arm_cmd_last_update = self.get_clock().now()
        self._reset_trajectory()
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None

    def _arm_joint_pos_cb(self, msg: Float64MultiArray) -> None:
        if self.arm_dim == 0:
            self.get_logger().warn("cmd/arm_joint_pos ignored because arm_dim is 0")
            return

        values = np.asarray(msg.data, dtype=float)
        if values.shape[0] < self.arm_dim:
            self.get_logger().warn(
                f"cmd/arm_joint_pos expects {self.arm_dim} values, got {values.shape[0]}"
            )
            return

        self.arm_joint_target = self._sanitize_arm_joint_target(values)
        self.arm_joint_target_set_time = self.get_clock().now()
        self._reset_trajectory()
        self.get_logger().info(f"Received arm joint target: {self.arm_joint_target.tolist()}")

    def _arm_joint_state_cb(self, msg: JointState) -> None:
        if self.arm_dim == 0:
            self.get_logger().warn("cmd/arm_joint_state ignored because arm_dim is 0")
            return
        if len(msg.position) == 0:
            self.get_logger().warn("cmd/arm_joint_state ignored because position is empty")
            return

        if len(msg.name) == 0:
            if len(msg.position) < self.arm_dim:
                self.get_logger().warn(
                    f"cmd/arm_joint_state expects at least {self.arm_dim} position values, "
                    f"got {len(msg.position)}"
                )
                return
            self.arm_joint_target = self._sanitize_arm_joint_target(
                np.asarray(msg.position[: self.arm_dim], dtype=float)
            )
            self.arm_joint_target_set_time = self.get_clock().now()
            self._reset_trajectory()
            self.get_logger().info(
                f"Received arm joint target (unnamed): {self.arm_joint_target.tolist()}"
            )
            return

        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        missing = [name for name in self._arm_joint_names if name not in name_to_pos]
        if missing:
            self.get_logger().warn(
                f"cmd/arm_joint_state missing joints: {missing}; expected {self._arm_joint_names}"
            )
            return

        self.arm_joint_target = self._sanitize_arm_joint_target(np.asarray(
            [name_to_pos[name] for name in self._arm_joint_names], dtype=float
        ))
        self.arm_joint_target_set_time = self.get_clock().now()
        self._reset_trajectory()
        self.get_logger().info(f"Received arm joint target (named): {self.arm_joint_target.tolist()}")

    def _gripper_cb(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=float)
        if values.shape[0] < self.gripper_dim:
            self.get_logger().warn(
                f"cmd/gripper expects {self.gripper_dim} values, got {values.shape[0]}"
            )
            return
        self.current_action[-self.gripper_dim :] = values[: self.gripper_dim]
        self._clip_action()
        self._gripper_cmd_count += 1
        if self._gripper_cmd_count <= 3 or self._gripper_cmd_count % 50 == 0:
            self.get_logger().info(
                f"Received gripper command: {self.current_action[-self.gripper_dim :].tolist()}"
            )

    def _publish_action_spec(self) -> None:
        payload: List[float] = [
            float(self.action_dim),
            float(self.arm_dim),
            float(self.gripper_dim),
        ]
        payload.extend(self.action_low.tolist())
        payload.extend(self.action_high.tolist())

        msg = Float64MultiArray()
        msg.data = payload
        self.action_spec_pub.publish(msg)

    def _publish_joint_state(self) -> None:
        sim = self.env.sim
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        names = []
        positions = []
        for joint_id in range(sim.model.njnt):
            name = sim.model.joint_id2name(joint_id)
            if name is None:
                name = f"joint_{joint_id}"
            qpos_addr = int(sim.model.jnt_qposadr[joint_id])
            names.append(name)
            positions.append(float(sim.data.qpos[qpos_addr]))

        msg.name = names
        msg.position = positions
        self.joint_state_pub.publish(msg)

    def _tick(self) -> None:
        if self._trajectory_points is not None and self._trajectory_start_time is not None:
            elapsed = (
                self.get_clock().now() - self._trajectory_start_time
            ).nanoseconds * 1e-9
            traj_pos, traj_vel, done = self._trajectory_target_at(elapsed)
            self.arm_joint_target = traj_pos
            self._trajectory_desired_vel = traj_vel
            self.arm_joint_target_set_time = None
            if done:
                self._reset_trajectory()
                self.arm_joint_target = traj_pos
                self.arm_joint_target_set_time = self.get_clock().now()
                self.get_logger().info("Joint trajectory finished")

        if self.arm_joint_target is not None and self.arm_dim > 0:
            current = self._get_arm_joint_positions()
            velocity = self._get_arm_joint_velocities()
            error = self.arm_joint_target - current
            arm_cmd = (
                self.arm_position_kp * error
                + 0.2 * self._trajectory_desired_vel
                - self.arm_position_kd * velocity
            )
            arm_cmd[np.abs(error) < self.arm_position_deadband] = 0.0
            self._set_arm_action_target(arm_cmd)

            reached = bool(
                np.max(np.abs(error)) < self.arm_target_reached_tol
                and np.max(np.abs(velocity)) < self.arm_target_reached_vel_tol
            )
            if reached:
                if self._trajectory_points is None:
                    self.arm_joint_target = None
                    self.arm_joint_target_set_time = None
                    self._set_arm_action_target(np.zeros(self.arm_dim, dtype=float))
                    self.get_logger().info("Arm joint target reached; holding still")
            elif self.arm_target_timeout > 0.0 and self.arm_joint_target_set_time is not None:
                elapsed = (
                    self.get_clock().now() - self.arm_joint_target_set_time
                ).nanoseconds * 1e-9
                if elapsed > self.arm_target_timeout:
                    self.arm_joint_target = None
                    self.arm_joint_target_set_time = None
                    self._set_arm_action_target(np.zeros(self.arm_dim, dtype=float))
                    self.get_logger().warn(
                        "Arm joint target timeout reached; stopping arm command"
                    )

        if self.arm_joint_target is None and self.arm_dim > 0:
            elapsed_direct = (
                self.get_clock().now() - self.arm_cmd_last_update
            ).nanoseconds * 1e-9
            if elapsed_direct > self.arm_cmd_timeout:
                self._set_arm_action_target(np.zeros(self.arm_dim, dtype=float))

        self._update_arm_action_cmd()
        self._clip_action()

        for _ in range(max(1, self.sim_steps_per_tick)):
            step_out = self.env.step(self.current_action)

            terminated = False
            truncated = False
            if isinstance(step_out, tuple):
                if len(step_out) >= 3:
                    terminated = bool(step_out[2])
                if len(step_out) >= 5:
                    truncated = bool(step_out[3])

            if terminated or truncated:
                if self.auto_reset_on_done:
                    self.get_logger().info("Episode finished, resetting environment")
                    self.env.reset()
                    self.current_action[:] = 0.0
                    self._clip_action()
                    break
                self.get_logger().warn(
                    "Episode finished but auto_reset_on_done=false; "
                    "set ignore_done=true for continuous simulation"
                )
                return

        if self.render:
            self.env.render()

        self._publish_joint_state()

    def _reset_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.env.reset()
        self.current_action[:] = 0.0
        self.arm_action_target[:] = 0.0
        self.arm_action_cmd[:] = 0.0
        self._reset_trajectory()
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        self.arm_cmd_last_update = self.get_clock().now()
        self._clip_action()
        response.success = True
        response.message = "Environment reset and action command zeroed"
        return response

    def _zero_action_cb(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.current_action[:] = 0.0
        self.arm_action_target[:] = 0.0
        self.arm_action_cmd[:] = 0.0
        self._reset_trajectory()
        self.arm_joint_target = None
        self.arm_joint_target_set_time = None
        self.arm_cmd_last_update = self.get_clock().now()
        self._clip_action()
        response.success = True
        response.message = "Action command set to zero"
        return response

    def destroy_node(self) -> bool:
        try:
            self.env.close()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f"Failed to close robosuite env cleanly: {exc}")
        try:
            self.follow_joint_trajectory_server.destroy()
        except Exception:
            pass
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobosuiteBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()