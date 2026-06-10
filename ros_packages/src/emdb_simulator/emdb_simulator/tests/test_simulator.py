#!/usr/bin/env python3
import numpy as np
import gymnasium as gym
import mujoco
import rclpy

from gymnasium import utils
from gymnasium.envs.mujoco import MujocoEnv
from gymnasium.spaces import Box
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_descriptions import panda_mj_description


class PandaGymEnv(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": ["human", "rgb_array", "depth_array", "rgbd_tuple"],
    }

    def __init__(self, render_mode=None):
        xml_file = panda_mj_description.MJCF_PATH
        frame_skip = 5

        utils.EzPickle.__init__(self, xml_file, frame_skip, render_mode)

        MujocoEnv.__init__(
            self,
            model_path=xml_file,
            frame_skip=frame_skip,
            observation_space=None,
            render_mode=render_mode,
            default_camera_config={},
        )

        obs_size = self.data.qpos.size + self.data.qvel.size
        self.observation_space = Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_size,),
            dtype=np.float64,
        )

        self.action_space = Box(
            low=-1.0,
            high=1.0,
            shape=(self.model.nu,),
            dtype=np.float32,
        )

        self.metadata = {
            "render_modes": ["human", "rgb_array", "depth_array", "rgbd_tuple"],
            "render_fps": int(np.round(1.0 / self.dt)),
        }

    def _get_obs(self):
        return np.concatenate([
            self.data.qpos.flat.copy(),
            self.data.qvel.flat.copy(),
        ])

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.do_simulation(action, self.frame_skip)

        obs = self._get_obs()
        reward = 0.0
        terminated = False
        truncated = False
        info = {}

        if self.render_mode == "human":
            self.render()

        return obs, reward, terminated, truncated, info

    def reset_model(self):
        qpos = self.init_qpos.copy()
        qvel = self.init_qvel.copy()
        self.set_state(qpos, qvel)
        return self._get_obs()


class GymMujocoRos2Bridge(Node):
    def __init__(self):
        super().__init__("gym_mujoco_bridge")

        self.declare_parameter("render_mode", "human")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("joint_state_topic", "/state/joint_states")
        self.declare_parameter("cmd_topic", "/cmd/action")

        joint_state_topic =  self.get_parameter("joint_state_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value

        render_mode = self.get_parameter("render_mode").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        

        if render_mode == "none":
            render_mode = None

        self.env = PandaGymEnv(render_mode=render_mode)
        self.obs, self.info = self.env.reset(seed=42)

        self.current_action = np.zeros(self.env.action_space.shape[0], dtype=np.float32)

        self.joint_pub = self.create_publisher(JointState, joint_state_topic, 10)
        self.create_subscription(
            Float64MultiArray,
            cmd_topic,
            self.cmd_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / rate_hz, self.step_env)

        self.get_logger().info(
            f"Started Panda Gym-MuJoCo ROS2 bridge | action_dim={self.env.action_space.shape[0]}"
        )

    def cmd_callback(self, msg: Float64MultiArray):
        action = np.array(msg.data, dtype=np.float32)
        if action.shape[0] != self.env.action_space.shape[0]:
            self.get_logger().warn(
                f"Expected action dim {self.env.action_space.shape[0]}, got {action.shape[0]}"
            )
            return

        self.current_action[:] = np.clip(
            action,
            self.env.action_space.low,
            self.env.action_space.high,
        )

    def step_env(self):
        self.obs, reward, terminated, truncated, info = self.env.step(self.current_action)
        self.publish_joint_state()

        if terminated or truncated:
            self.get_logger().info("Episode finished, resetting env")
            self.obs, self.info = self.env.reset()

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        model = self.env.unwrapped.model
        data = self.env.unwrapped.data

        names = []
        positions = []
        velocities = []

        for j in range(model.njnt):
            jnt_type = model.jnt_type[j]

            if jnt_type == mujoco.mjtJoint.mjJNT_FREE:
                continue
            if jnt_type == mujoco.mjtJoint.mjJNT_BALL:
                continue

            qpos_adr = model.jnt_qposadr[j]
            qvel_adr = model.jnt_dofadr[j]

            try:
                name = model.joint(j).name
            except Exception:
                name = None

            if not name:
                name = f"joint_{j}"

            names.append(name)
            positions.append(float(data.qpos[qpos_adr]))
            velocities.append(float(data.qvel[qvel_adr]))

        msg.name = names
        msg.position = positions
        msg.velocity = velocities
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GymMujocoRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        env = node.env.unwrapped
        env.close()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()