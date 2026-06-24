#!/usr/bin/env python3
import json
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.node import Node

import robosuite
from robosuite.controllers import load_composite_controller_config

import robocasa.macros as macros
from robocasa.models.scenes.scene_registry import LayoutType, StyleType
from robocasa.scripts.collect_demos import collect_human_trajectory
from robocasa.wrappers.enclosing_wall_render_wrapper import (
    EnclosingWallRenderWrapper,
    install_enclosing_wall_hotkeys,
)


class RoboCasaTeleopNode(Node):
    def __init__(self):
        super().__init__("robocasa_teleop_node")

        self.declare_parameter("task", "Kitchen")
        self.declare_parameter("robot", "Tiago")
        self.declare_parameter("layout_id", 3)
        self.declare_parameter("style_id", 1)
        self.declare_parameter("device", "keyboard")
        self.declare_parameter("show_walls", False)
        self.declare_parameter("renderer", "mjviewer")
        self.declare_parameter("control_freq", 20.0)

        self.task = self.get_parameter("task").value
        self.robot = self.get_parameter("robot").value
        self.layout_id = int(self.get_parameter("layout_id").value)
        self.style_id = int(self.get_parameter("style_id").value)
        self.device_name = self.get_parameter("device").value
        self.show_walls = bool(self.get_parameter("show_walls").value)
        self.renderer = self.get_parameter("renderer").value
        self.control_freq = float(self.get_parameter("control_freq").value)

        self.layouts = self._build_layouts()
        self.styles = self._build_styles()

        self.env = None
        self.device = None

        self._create_env()
        self._set_layout_style()
        self._create_device()

        self.get_logger().info("Starting teleoperation / demo collection")
        self.timer = self.create_timer(0.5, self._run_once)
        self._started = False

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
            "controller_configs": load_composite_controller_config(robot=self.robot),
            "translucent_robot": False,
            "layout_ids": [self.layout_id],
            "style_ids": [self.style_id],
        }

        self.get_logger().info("Initializing RoboCasa environment...")
        self.get_logger().info(json.dumps(config))

        env = robosuite.make(
            **config,
            has_renderer=True,
            has_offscreen_renderer=False,
            render_camera=None,
            ignore_done=True,
            use_camera_obs=False,
            control_freq=self.control_freq,
            renderer=self.renderer,
        )

        env = EnclosingWallRenderWrapper(env, alpha=0.1, enabled=not self.show_walls)
        install_enclosing_wall_hotkeys(env)
        self.env = env

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

        self.get_logger().debug(
            f"Scene selected -> Layout: {layout} ({self.layouts.get(layout, 'unknown')}), "
            f"Style: {style} ({self.styles.get(style, 'unknown')})"
        )

    def _create_device(self):
        if self.device_name == "keyboard":
            from robosuite.devices import Keyboard
            self.device = Keyboard(
                env=self.env,
                pos_sensitivity=4.0,
                rot_sensitivity=4.0,
            )
        elif self.device_name == "spacemouse":
            from robosuite.devices import SpaceMouse
            self.device = SpaceMouse(
                env=self.env,
                pos_sensitivity=4.0,
                rot_sensitivity=4.0,
                vendor_id=macros.SPACEMOUSE_VENDOR_ID,
                product_id=macros.SPACEMOUSE_PRODUCT_ID,
            )
        else:
            raise ValueError(f"Unsupported device: {self.device_name}")

        self.get_logger().info(f"Input device initialized: {self.device_name}")

    def _run_once(self):
        if self._started:
            return
        self._started = True
        self.timer.cancel()

        try:
            collect_human_trajectory(
                self.env,
                self.device,
                "right",
                "single-arm-opposed",
                mirror_actions=True,
                render=(self.renderer != "mjviewer"),
                max_fr=30,
                print_info=False,
            )
            self.get_logger().info("Trajectory collection / teleop session finished")
        except Exception as e:
            self.get_logger().error(f"Teleop failed: {e}")
        finally:
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
    node = RoboCasaTeleopNode()
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