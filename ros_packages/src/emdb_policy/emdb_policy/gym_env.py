#!/usr/bin/env python3
"""gymnasium.Env wrapper around AgentBridge, for use with stable-baselines3 etc.

Requires rclpy.init() to have already been called by the caller. Talks to
the simulator purely through /observations, /reward, /step_action and
/reset_episode -- no dependency on robosuite/robocasa/mujoco in this process.
"""
import numpy as np
import gymnasium as gym
from gymnasium import spaces

from emdb_policy.agent_bridge import AgentBridge

# [dx, dy, dz, droll, dpitch, dyaw, gripper]
ACTION_DIM = 7


class EmdbGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        node_name="emdb_gym_env",
        max_episode_steps=200,
        pos_scale=0.05,
        rot_scale=0.5,
        layout_id=-1,
        style_id=-1,
        service_timeout_sec=10.0,
        reset_timeout_sec=90.0,
    ):
        super().__init__()

        self.max_episode_steps = max_episode_steps
        self.pos_scale = pos_scale
        self.rot_scale = rot_scale
        self.layout_id = layout_id
        self.style_id = style_id
        self._episode_step = 0

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(ACTION_DIM,), dtype=np.float32)
        self._obs_keys = None

        self.bridge = AgentBridge(
            node_name=node_name,
            step_timeout_sec=service_timeout_sec,
            reset_timeout_sec=reset_timeout_sec,
        )
        self.bridge.start()
        self.bridge.wait_for_services()

        # Discover the flattened obs layout (keys + total dim) from a real reset,
        # since RoboCasa's obs_dict keys/shapes depend on the running task/robot.
        obs_dict = self.bridge.reset(layout_id=self.layout_id, style_id=self.style_id)
        self._obs_keys = sorted(obs_dict.keys())
        flat_obs = self._flatten_obs(obs_dict)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=flat_obs.shape, dtype=np.float32
        )
        self._episode_step = 0

    def _flatten_obs(self, obs_dict):
        missing = [k for k in self._obs_keys if k not in obs_dict]
        if missing:
            raise RuntimeError(
                f"Observation is missing expected keys {missing}; the running task/robot "
                "must not change after EmdbGymEnv is constructed."
            )
        return np.concatenate(
            [np.asarray(obs_dict[k], dtype=np.float32).flatten() for k in self._obs_keys]
        )

    def _scale_action(self, action):
        scaled = np.asarray(action, dtype=np.float64).copy()
        scaled[0:3] *= self.pos_scale
        scaled[3:6] *= self.rot_scale
        return scaled

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        layout_id = (options or {}).get("layout_id", self.layout_id)
        style_id = (options or {}).get("style_id", self.style_id)

        obs_dict = self.bridge.reset(layout_id=layout_id, style_id=style_id)
        self._episode_step = 0
        return self._flatten_obs(obs_dict), {}

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        obs_dict, reward, terminated, truncated, info = self.bridge.step_vector(
            self._scale_action(action)
        )

        self._episode_step += 1
        if self._episode_step >= self.max_episode_steps:
            truncated = True

        return self._flatten_obs(obs_dict), float(reward), bool(terminated), bool(truncated), info

    def render(self):
        pass

    def close(self):
        self.bridge.close()
