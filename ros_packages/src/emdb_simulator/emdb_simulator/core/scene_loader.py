#!/usr/bin/env python3
import json
from copy import deepcopy
from collections import OrderedDict
import traceback

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
from emdb_simulator.core.ros_keyboard_device import ROSKeyboardDevice


class SceneLoader(Node):
    def __init__(self):
        super().__init__("robocasa_rollout_node")

        self.declare_parameter("task", "Kitchen")
        self.declare_parameter("robot", "PandaOmron")
        self.declare_parameter("layout_id", 2)
        self.declare_parameter("style_id", 1)
        self.declare_parameter("show_walls", False)
        self.declare_parameter("renderer", "mjviewer")
        self.declare_parameter("publish_rate", 20.0)

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

        self.device = None
        self.all_prev_gripper_actions = None
        self.mirror_actions = True

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
        self._init_device()

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
            control_freq=int(self.publish_rate),
            renderer=self.renderer,
        )

        self.env = EnclosingWallRenderWrapper(
            self.env, alpha=0.1, enabled=not self.show_walls
        )
        install_enclosing_wall_hotkeys(self.env)
        self.env.reset()

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

        self.get_logger().info(
            f"Joint mapping initialized with {len(self.robot_joint_names)} joints"
        )

    def _init_device(self):
        self.device = ROSKeyboardDevice(
            env=self.env,
            pos_sensitivity=1.0,
            rot_sensitivity=1.0,
        )
        self.device.start_control()

        self.all_prev_gripper_actions = [
            {
                f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
                for robot_arm in robot.arms
                if robot.gripper[robot_arm].dof > 0
            }
            for robot in self.env.robots
        ]

    def _reset_env_cb(self, request, response):
        try:
            self.env.reset()
            self._set_layout_style()
            self.device.start_control()
            self._init_device()
            response.success = True
            response.message = "Environment reset successfully"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to reset environment: {e}"
            self.get_logger().error(response.message)
        return response

    def _set_delta_action_cb(self, request, response):
        if request.toggle_base_mode:
            self.device.toggle_base_mode()

        if request.grasp:
            self.device.toggle_grasp()

        if request.reset:
            self.device.trigger_reset()
            response.success = True
            response.message = "Reset solicitado"
            return response

        if hasattr(request, "next_arm") and request.next_arm:
            self.device.next_arm()

        if hasattr(request, "next_robot") and request.next_robot:
            self.device.next_robot()

        if self.device.base_mode:
            self.device.apply_delta(
                dx=request.base_dx,
                dy=request.base_dy,
                dyaw=request.base_dyaw,
            )
        else:
            self.device.apply_delta(
                dx=request.dx,
                dy=request.dy,
                dz=request.dz,
                droll=request.droll,
                dpitch=request.dpitch,
                dyaw=request.dyaw,
            )

        response.success = True
        response.message = (
            f"Delta recibido | robot={self.device.active_robot} "
            f"arm={self.device.active_arm} "
            f"base_mode={int(self.device.base_mode)} "
            f"grasp={int(self.device.grasp)}"
        )
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
            active_robot = self.env.robots[self.device.active_robot]
            input_ac_dict = self.device.input2action(mirror_actions=self.mirror_actions)
            self.get_logger().debug(f"input_ac_dict={input_ac_dict}")

            if input_ac_dict is None:
                self.env.reset()
                self.device.start_control()
                self.device.clear_reset()
                return

            action_dict = deepcopy(input_ac_dict)

            for arm in active_robot.arms:
                controller_input_type = active_robot.part_controllers[arm].input_type
                if controller_input_type == "delta":
                    action_dict[arm] = input_ac_dict[f"{arm}_delta"]
                elif controller_input_type == "absolute":
                    action_dict[arm] = input_ac_dict[f"{arm}_abs"]
                else:
                    raise ValueError(f"Unsupported input_type: {controller_input_type}")

            env_action = [
                robot.create_action_vector(self.all_prev_gripper_actions[i])
                for i, robot in enumerate(self.env.robots)
            ]

            env_action[self.device.active_robot] = active_robot.create_action_vector(action_dict)

            for arm in active_robot.arms:
                key = f"{arm}_gripper"
                if key in action_dict:
                    self.all_prev_gripper_actions[self.device.active_robot][key] = action_dict[key]

            env_action = np.concatenate(env_action)

            self.env.step(env_action)
            self._publish_joint_states()
            self.env.render()

            if self.device._reset_state:
                self.env.reset()
                self.device.start_control()
                self.device.clear_reset()

        except Exception as e:
            self.get_logger().error(f"Render / control failed: {e}")
            self.get_logger().error(traceback.format_exc())
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