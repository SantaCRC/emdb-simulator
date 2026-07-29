#!/usr/bin/env python3
import json
import os
import time
from copy import deepcopy
from collections import OrderedDict
from glob import glob
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

import robosuite
import robocasa
from robosuite.environments.base import REGISTERED_ENVS
from robocasa.environments.kitchen.kitchen import Kitchen
from robocasa.models.scenes.scene_registry import LayoutType, StyleType
from robocasa.wrappers.enclosing_wall_render_wrapper import (
    EnclosingWallRenderWrapper,
    install_enclosing_wall_hotkeys,
)
from robosuite.wrappers import DataCollectionWrapper
from robocasa.scripts.collect_demos import gather_demonstrations_as_hdf5
from robocasa.utils.robomimic.robomimic_dataset_utils import convert_to_robomimic_format

from emdb_interfaces.srv import (
    SetDeltaAction,
    StepAction,
    StepActionRaw,
    ResetEpisode,
    SaveDemos,
)
from emdb_simulator.core import robot_loader  # noqa: F401  registers UR5eOmron with robosuite
from emdb_simulator.core.ros_keyboard_device import ROSKeyboardDevice
from emdb_interfaces.msg import (
    ObjectState,
    ObjectStateArray,
    Observation,
    ObservationEntry,
    StepInfo,
)



class SceneLoader(Node):
    def __init__(self):
        super().__init__("robocasa_rollout_node")

        self.declare_parameter("task", "PickPlaceCounterToCabinet")
        self.declare_parameter("robot", "UR5eOmron")
        # layout_id 2 (and every "test" layout, 1-10) places an open_cabinet
        # fixture that PickPlaceCounterToCabinet can pick as its target "cab",
        # which MimicGen's cabinet geom lookup (assumes a boxed cabinet) can't
        # handle. "train" layouts 11-60 were checked and never include one.
        self.declare_parameter("layout_id", 11)
        self.declare_parameter("style_id", 11)
        self.declare_parameter("show_walls", False)
        self.declare_parameter("renderer", "mjviewer")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("control_mode", "teleop")
        self.declare_parameter("collect_demos", False)
        self.declare_parameter("demo_dir", "/tmp/emdb_demos")

        self.task = self.get_parameter("task").value
        self.robot = self.get_parameter("robot").value
        self.layout_id = int(self.get_parameter("layout_id").value)
        self.style_id = int(self.get_parameter("style_id").value)
        self.show_walls = bool(self.get_parameter("show_walls").value)
        self.renderer = self.get_parameter("renderer").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.control_mode = self.get_parameter("control_mode").value
        self.collect_demos = bool(self.get_parameter("collect_demos").value)
        self.demo_dir = self.get_parameter("demo_dir").value
        self.demo_tmp_directory = None
        self.env_info = None

        self.episode_id = 0
        self.step_id = 0

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

        self.object_state_pub = self.create_publisher(ObjectStateArray, "/object_states", 10)

        self.observation_pub = self.create_publisher(Observation, "/observations", 10)

        self.step_info_pub = self.create_publisher(StepInfo, "/reward", 10)

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

        self.step_action_srv = self.create_service(
            StepAction,
            "/step_action",
            self._step_action_cb,
        )

        self.step_action_raw_srv = self.create_service(
            StepActionRaw,
            "/step_action_raw",
            self._step_action_raw_cb,
        )

        self.reset_episode_srv = self.create_service(
            ResetEpisode,
            "/reset_episode",
            self._reset_episode_cb,
        )

        self.save_demos_srv = self.create_service(
            SaveDemos,
            "/save_demos",
            self._save_demos_cb,
        )

        self._create_env()
        self._set_layout_style()
        self._init_robot_joint_mapping()
        self._init_device()

        self.get_logger().info(
            f"Scene loaded -> layout={self.current_layout}, style={self.current_style}"
        )

        if self.control_mode == "teleop":
            self.timer = self.create_timer(1.0 / self.publish_rate, self._render_loop)
        else:
            self.timer = None
            self.get_logger().info(
                "control_mode=rl -> periodic render loop disabled; "
                "physics only advances on /step_action calls"
            )

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
        self.is_kitchen_task = issubclass(REGISTERED_ENVS[self.task], Kitchen)

        config = {
            "env_name": self.task,
            "robots": self.robot,
        }
        if self.is_kitchen_task:
            config["translucent_robot"] = False
            config["layout_ids"] = [self.layout_id]
            config["style_ids"] = [self.style_id]

        self.get_logger().info(
            f"Initializing {'RoboCasa' if self.is_kitchen_task else 'robosuite'} scene..."
        )
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

        if self.is_kitchen_task:
            self.env = EnclosingWallRenderWrapper(
                self.env, alpha=0.1, enabled=not self.show_walls
            )
            install_enclosing_wall_hotkeys(self.env)

        self.env_info = json.dumps(config)
        if self.collect_demos:
            t1, t2 = str(time.time()).split(".")
            self.demo_tmp_directory = f"/tmp/emdb_demo_raw_{t1}_{t2}"
            self.env = DataCollectionWrapper(
                self.env, self.demo_tmp_directory, use_env_xml_for_reset=True
            )
            self.get_logger().info(
                f"collect_demos=true -> recording teleop episodes to {self.demo_tmp_directory}"
            )

        self.env.reset()

    def _set_layout_style(self):
        if not self.is_kitchen_task:
            self.current_layout = None
            self.current_style = None
            return

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
            self.episode_id += 1
            self.step_id = 0
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

    def _build_env_action(self, active_robot, input_ac_dict):
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

        return np.concatenate(env_action)

    def _translate_delta_to_env_action(self, request):
        if request.next_arm:
            self.device.next_arm()

        if request.next_robot:
            self.device.next_robot()

        if request.grasp:
            self.device.toggle_grasp()

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

        active_robot = self.env.robots[self.device.active_robot]
        input_ac_dict = self.device.input2action(mirror_actions=self.mirror_actions)
        if input_ac_dict is None:
            raise RuntimeError(
                "device.input2action() returned None (reset pending) during an RL step"
            )

        return self._build_env_action(active_robot, input_ac_dict)

    def _publish_observation(self, obs_dict):
        msg = Observation()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.episode_id = self.episode_id
        msg.step_id = self.step_id

        entries = []
        for key, value in obs_dict.items():
            arr = np.asarray(value)
            if not np.issubdtype(arr.dtype, np.number):
                continue
            entry = ObservationEntry()
            entry.key = key
            entry.data = arr.astype(np.float64).flatten().tolist()
            entry.shape = [int(d) for d in arr.shape]
            entries.append(entry)

        msg.entries = entries
        self.observation_pub.publish(msg)

    def _publish_step_info(self, reward, terminated, truncated, success):
        msg = StepInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.episode_id = self.episode_id
        msg.step_id = self.step_id
        msg.reward = float(reward)
        msg.terminated = bool(terminated)
        msg.truncated = bool(truncated)
        msg.success = bool(success)
        self.step_info_pub.publish(msg)

    def _apply_env_action_and_publish(self, env_action):
        obs, reward, _done, _info = self.env.step(env_action)
        success = bool(self.env._check_success())
        self.step_id += 1

        self._publish_joint_states()
        self._publish_object_states()
        self._publish_observation(obs)
        self._publish_step_info(reward, terminated=success, truncated=False, success=success)
        self.env.render()

    def _step_action_cb(self, request, response):
        if self.control_mode != "rl":
            response.success = False
            response.message = "control_mode is 'teleop'; /step_action is only available in 'rl' mode"
            response.episode_id = self.episode_id
            response.step_id = self.step_id
            return response

        try:
            env_action = self._translate_delta_to_env_action(request)
            self._apply_env_action_and_publish(env_action)
            response.success = True
            response.message = "ok"
        except Exception as e:
            self.get_logger().error(f"/step_action failed: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Step failed: {e}"

        response.episode_id = self.episode_id
        response.step_id = self.step_id
        return response

    def _step_action_raw_cb(self, request, response):
        if self.control_mode != "rl":
            response.success = False
            response.message = (
                "control_mode is 'teleop'; /step_action_raw is only available in 'rl' mode"
            )
            response.episode_id = self.episode_id
            response.step_id = self.step_id
            return response

        try:
            env_action = np.asarray(request.action, dtype=np.float64)
            self._apply_env_action_and_publish(env_action)
            response.success = True
            response.message = "ok"
        except Exception as e:
            self.get_logger().error(f"/step_action_raw failed: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Step failed: {e}"

        response.episode_id = self.episode_id
        response.step_id = self.step_id
        return response

    def _reset_episode_cb(self, request, response):
        try:
            if request.layout_id != -1:
                self.layout_id = int(request.layout_id)
            if request.style_id != -1:
                self.style_id = int(request.style_id)

            self._set_layout_style()
            obs = self.env.reset()
            self.device.start_control()
            self.all_prev_gripper_actions = [
                {
                    f"{robot_arm}_gripper": np.repeat([0], robot.gripper[robot_arm].dof)
                    for robot_arm in robot.arms
                    if robot.gripper[robot_arm].dof > 0
                }
                for robot in self.env.robots
            ]

            self.episode_id += 1
            self.step_id = 0

            self._publish_joint_states()
            self._publish_object_states()
            self._publish_observation(obs)
            self._publish_step_info(reward=0.0, terminated=False, truncated=False, success=False)

            response.success = True
            response.message = (
                f"Episode reset -> layout={self.current_layout}, style={self.current_style}"
            )
        except Exception as e:
            self.get_logger().error(f"/reset_episode failed: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Reset failed: {e}"

        response.episode_id = self.episode_id
        return response

    def _find_successful_episodes(self, directory):
        successful = []
        for ep_directory in os.listdir(directory):
            state_paths = os.path.join(directory, ep_directory, "state_*.npz")
            success = False
            for state_file in sorted(glob(state_paths)):
                dic = np.load(state_file, allow_pickle=True)
                success = success or bool(dic["successful"])
            if success:
                successful.append(ep_directory)
        return successful

    def _save_demos_cb(self, request, response):
        if not self.collect_demos:
            response.success = False
            response.message = "collect_demos param is false; no episodes are being recorded"
            response.hdf5_path = ""
            return response

        try:
            out_dir = request.out_dir if request.out_dir else self.demo_dir
            os.makedirs(out_dir, exist_ok=True)

            successful_episodes = self._find_successful_episodes(self.demo_tmp_directory)
            hdf5_path = gather_demonstrations_as_hdf5(
                self.demo_tmp_directory,
                out_dir,
                self.env_info,
                successful_episodes=successful_episodes,
                verbose=True,
            )

            if hdf5_path is None:
                response.success = False
                response.message = "No successful episodes recorded yet"
                response.hdf5_path = ""
                return response

            convert_to_robomimic_format(hdf5_path)

            response.success = True
            response.message = (
                f"Saved {len(successful_episodes)} successful demo(s) to {hdf5_path}"
            )
            response.hdf5_path = hdf5_path
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error(f"/save_demos failed: {e}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.message = f"Failed to save demos: {e}"
            response.hdf5_path = ""

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

    def _publish_object_states(self):
        sim = self.env.sim
        msg = ObjectStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.objects = []

        for i in range(sim.model.njnt):
            name = sim.model.joint_id2name(i)
            if not name or ("obj" not in name and "distr" not in name):
                continue

            qpos_addr = sim.model.get_joint_qpos_addr(name)
            if isinstance(qpos_addr, tuple):
                qpos = sim.data.get_joint_qpos(name)

                obj = ObjectState()
                obj.name = name
                obj.pose.position.x = float(qpos[0])
                obj.pose.position.y = float(qpos[1])
                obj.pose.position.z = float(qpos[2])
                obj.pose.orientation.w = float(qpos[3])
                obj.pose.orientation.x = float(qpos[4])
                obj.pose.orientation.y = float(qpos[5])
                obj.pose.orientation.z = float(qpos[6])

                msg.objects.append(obj)

        self.object_state_pub.publish(msg)


    def _render_loop(self):
        try:
            active_robot = self.env.robots[self.device.active_robot]
            input_ac_dict = self.device.input2action(mirror_actions=self.mirror_actions)
            self.get_logger().debug(f"input_ac_dict={input_ac_dict}")
            sim_model = self.env.sim.model
            for i in range(sim_model.njnt):
                name = sim_model.joint_id2name(i)
                if name and ("obj" in name or "distr" in name):
                    self.get_logger().debug(f"Joint {i}: {name}")
            if self.env._check_success():
                self.get_logger().info("Success achieved!")
                # Update success metrics

            if input_ac_dict is None:
                self.env.reset()
                self.device.start_control()
                self.device.clear_reset()
                self.episode_id += 1
                self.step_id = 0
                return

            env_action = self._build_env_action(active_robot, input_ac_dict)
            self._apply_env_action_and_publish(env_action)

            if self.device._reset_state:
                self.env.reset()
                self.device.start_control()
                self.device.clear_reset()
                self.episode_id += 1
                self.step_id = 0

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