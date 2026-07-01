#!/usr/bin/env python3
import json
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import robosuite
import robocasa
from robocasa.models.scenes.scene_registry import LayoutType, StyleType
from robocasa.wrappers.enclosing_wall_render_wrapper import (
    EnclosingWallRenderWrapper,
    install_enclosing_wall_hotkeys,
)

from emdb_interfaces.srv import SetDeltaAction


class SceneLoader(Node):
    def __init__(self):
        super().__init__("robocasa_rollout_node")

        self.declare_parameter("task", "Kitchen")
        self.declare_parameter("robot", "PandaOmron")
        self.declare_parameter("layout_id", 2)
        self.declare_parameter("style_id", 1)
        self.declare_parameter("show_walls", False)
        self.declare_parameter("renderer", "mjviewer")
        self.declare_parameter("publish_rate", 30.0)

        self.task = self.get_parameter("task").value
        self.robot = self.get_parameter("robot").value
        self.layout_id = int(self.get_parameter("layout_id").value)
        self.style_id = int(self.get_parameter("style_id").value)
        self.show_walls = bool(self.get_parameter("show_walls").value)
        self.renderer = self.get_parameter("renderer").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.layouts = self._build_layouts()
        self.styles = self._build_styles()
        self.env = None
        self.robot_model = None

        self.robot_joint_names = []
        self.robot_qpos_idx = []
        self.robot_qvel_idx = []

        self.action_low = None
        self.action_high = None
        self.current_action = None
        self.grasp_state = 0.0

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.set_delta_srv = self.create_service(
            SetDeltaAction,
            "/set_delta_action",
            self._set_delta_action_cb,
        )

        self.reset_env_srv = self.create_service(
            Trigger,
            "/reset_env",
            self._reset_env_cb,
        )

        self._create_env()
        self._set_layout_style()
        self._init_robot_joint_mapping()
        self._init_action_interface()

        self.get_logger().info(
            f"Scene loaded -> layout={self.current_layout}, style={self.current_style}"
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self._render_loop)

    def _build_layouts(self):
        raw = dict((item.value, item.name.lower().capitalize()) for item in LayoutType)
        ordered = OrderedDict()
        for k in sorted(raw.keys()):
            if k < 0:
                continue
            ordered[int(k)] = raw[k]
        return ordered

    def _build_styles(self):
        raw = dict((item.value, item.name.lower().capitalize()) for item in StyleType)
        ordered = OrderedDict()
        for k in sorted(raw.keys()):
            if k < 0:
                continue
            ordered[int(k)] = raw[k]
        return ordered

    def _create_env(self):
        config = {
            "env_name": self.task,
            "robots": self.robot,
            "translucent_robot": False,
            "layout_ids": [self.layout_id],
            "style_ids": [self.style_id],
        }

        self.get_logger().info("Initializing RoboCasa scene...")
        self.get_logger().info(json.dumps(config))

        self.env = robosuite.make(
            **config,
            has_renderer=True,
            has_offscreen_renderer=False,
            render_camera=None,
            ignore_done=True,
            use_camera_obs=False,
            renderer=self.renderer,
        )

        self.env = EnclosingWallRenderWrapper(
            self.env, alpha=0.1, enabled=not self.show_walls
        )
        install_enclosing_wall_hotkeys(self.env)
        self.env.reset()

    def _reset_env_cb(self, request, response):
        try:
            self.env.reset()
            self._set_layout_style()
            self.current_action = np.zeros_like(self.current_action)
            self.grasp_state = 0.0
            response.success = True
            response.message = "Environment reset successfully"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset environment: {e}"
            self.get_logger().error(response.message)
        return response

    def _set_layout_style(self):
        layout = self.layout_id
        style = self.style_id

        if layout == -1:
            layout = int(np.random.choice(list(self.layouts.keys())))
        if style == -1:
            style = int(np.random.choice(list(self.styles.keys())))

        self.env.layout_and_style_ids = [[layout, style]]
        self.current_layout = layout
        self.current_style = style

    def _init_robot_joint_mapping(self):
        if not hasattr(self.env, "robots") or len(self.env.robots) == 0:
            raise RuntimeError("No robots found in environment")

        self.robot_model = self.env.robots[0]

        if hasattr(self.robot_model, "robot_model") and hasattr(self.robot_model.robot_model, "joints"):
            self.robot_joint_names = list(self.robot_model.robot_model.joints)
        elif hasattr(self.robot_model, "joints"):
            self.robot_joint_names = list(self.robot_model.joints)
        elif hasattr(self.robot_model, "_ref_joint_indexes"):
            idxs = list(self.robot_model._ref_joint_indexes)
            self.robot_joint_names = [self.env.sim.model.joint_id2name(i) for i in idxs]
        else:
            raise RuntimeError("Could not determine robot joint names")

        sim_model = self.env.sim.model
        qpos_idx, qvel_idx, valid_names = [], [], []

        for name in self.robot_joint_names:
            try:
                qpos_addr = sim_model.get_joint_qpos_addr(name)
                qvel_addr = sim_model.get_joint_qvel_addr(name)
                if isinstance(qpos_addr, tuple) or isinstance(qvel_addr, tuple):
                    continue
                valid_names.append(name)
                qpos_idx.append(int(qpos_addr))
                qvel_idx.append(int(qvel_addr))
            except Exception:
                continue

        self.robot_joint_names = valid_names
        self.robot_qpos_idx = qpos_idx
        self.robot_qvel_idx = qvel_idx

        if len(self.robot_joint_names) == 0:
            raise RuntimeError("No valid 1-DoF robot joints found")

        self.get_logger().info(f"Joint mapping initialized with {len(self.robot_joint_names)} joints")

    def _init_action_interface(self):
        low, high = self.env.action_spec
        self.action_low = np.asarray(low, dtype=np.float64)
        self.action_high = np.asarray(high, dtype=np.float64)
        self.current_action = np.zeros_like(self.action_low)
        self.get_logger().info(f"Action dimension: {len(self.current_action)}")

    def _set_delta_action_cb(self, request, response):
        if request.reset:
            self.env.reset()
            self.current_action = np.zeros_like(self.current_action)
            self.grasp_state = 0.0
            response.success = True
            response.message = "Reset aplicado"
            return response

        action = np.zeros_like(self.current_action)

        if len(action) >= 3:
            action[0] = request.dx
            action[1] = request.dy
            action[2] = request.dz

        if len(action) >= 6:
            action[3] = request.droll
            action[4] = request.dpitch
            action[5] = request.dyaw

        if request.grasp:
            self.grasp_state = -1.0 if self.grasp_state > 0.0 else 1.0

        if len(action) >= 7:
            action[6] = self.grasp_state

        self.current_action = np.clip(action, self.action_low, self.action_high)

        response.success = True
        response.message = f"Delta aplicado: {self.current_action.tolist()}"
        self.get_logger().info(response.message)
        return response

    def _publish_joint_states(self):
        sim = self.env.sim
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.robot_joint_names)
        msg.position = [float(sim.data.qpos[i]) for i in self.robot_qpos_idx]
        msg.velocity = [float(sim.data.qvel[i]) for i in self.robot_qvel_idx]
        msg.effort = []
        self.joint_state_pub.publish(msg)

    def _render_loop(self):
        try:
            self.env.step(self.current_action)
            self._publish_joint_states()
            self.env.render()
            self.current_action = np.zeros_like(self.current_action)
            if len(self.current_action) >= 7:
                self.current_action[6] = self.grasp_state
        except Exception as e:
            self.get_logger().error(f"Render / control failed: {e}")
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SceneLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()