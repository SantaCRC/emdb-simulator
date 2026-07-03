#!/usr/bin/env python3
import numpy as np
from robosuite.devices import Device
from robosuite.utils.transform_utils import rotation_matrix


class ROSKeyboardDevice(Device):
    def __init__(self, env, pos_sensitivity=1.0, rot_sensitivity=1.0):
        super().__init__(env)

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        self._reset_state = 0
        self._enabled = False
        self._pos_step = 0.05

        self._reset_internal_state()

    def _reset_internal_state(self):
        super()._reset_internal_state()

        self.rotation = np.array([
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        self.raw_drotation = np.zeros(3)
        self.last_drotation = np.zeros(3)
        self.pos = np.zeros(3)
        self.last_pos = np.zeros(3)

    def start_control(self):
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        dpos = self.pos - self.last_pos
        self.last_pos = np.array(self.pos)

        raw_drotation = self.raw_drotation - self.last_drotation
        self.last_drotation = np.array(self.raw_drotation)

        return dict(
            dpos=dpos,
            rotation=self.rotation,
            raw_drotation=raw_drotation,
            grasp=int(self.grasp),
            reset=self._reset_state,
            base_mode=int(self.base_mode),
        )

    def _postprocess_device_outputs(self, dpos, drotation):
        drotation = drotation * 1.5
        dpos = dpos * 75.0

        dpos = np.clip(dpos, -1.0, 1.0)
        drotation = np.clip(drotation, -1.0, 1.0)

        return dpos, drotation

    def apply_delta(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0):
        if not self._enabled:
            return

        self.pos[0] += dx * self.pos_sensitivity
        self.pos[1] += dy * self.pos_sensitivity
        self.pos[2] += dz * self.pos_sensitivity

        if droll != 0.0:
            drot = rotation_matrix(
                angle=droll * self.rot_sensitivity,
                direction=[1.0, 0.0, 0.0]
            )[:3, :3]
            self.rotation = self.rotation.dot(drot)
            self.raw_drotation[1] += droll * self.rot_sensitivity

        if dpitch != 0.0:
            drot = rotation_matrix(
                angle=dpitch * self.rot_sensitivity,
                direction=[0.0, 1.0, 0.0]
            )[:3, :3]
            self.rotation = self.rotation.dot(drot)
            self.raw_drotation[0] += dpitch * self.rot_sensitivity

        if dyaw != 0.0:
            drot = rotation_matrix(
                angle=dyaw * self.rot_sensitivity,
                direction=[0.0, 0.0, 1.0]
            )[:3, :3]
            self.rotation = self.rotation.dot(drot)
            self.raw_drotation[2] += dyaw * self.rot_sensitivity

    def toggle_grasp(self):
        self.grasp_states[self.active_robot][self.active_arm_index] = \
            not self.grasp_states[self.active_robot][self.active_arm_index]

    def toggle_base_mode(self):
        self.base_modes[self.active_robot] = \
            not self.base_modes[self.active_robot]

    def trigger_reset(self):
        self._reset_state = 1
        self._enabled = False
        self._reset_internal_state()

    def next_arm(self):
        self.active_arm_index = (
            self.active_arm_index + 1
        ) % len(self.all_robot_arms[self.active_robot])

    def next_robot(self):
        self.active_robot = (self.active_robot + 1) % self.num_robots

    def clear_reset(self):
        self._reset_state = 0