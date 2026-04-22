"""Gripper grasp test demo for the QbHand2M hand.

This script builds a minimal MuJoCo scene with a table, a box object,
and a QbHand2M gripper mounted on a vertical slide joint so the hand can
approach, close, lift, and release the object.

Run:
	$ python -m robosuite_ros.test_pick
"""

from __future__ import annotations

import os
import shutil
import tempfile
from importlib import resources
from pathlib import Path
import xml.etree.ElementTree as ET

from robosuite.models import MujocoWorldBase
from robosuite.models.arenas.table_arena import TableArena
from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.models.objects import BoxObject
from robosuite.renderers.viewer import OpenCVViewer
from robosuite.utils.binding_utils import MjRenderContextOffscreen, MjSim
from robosuite.utils.mjcf_utils import new_actuator, new_joint

from robosuite_ros.grippers.qbhand2m_gripper import QbHand2MGripperBase


def _contact_enabled_gripper_xml_path() -> Path:
	"""Materialize a contact-enabled copy of the packaged gripper XML."""

	models_root = resources.files("robosuite_ros").joinpath("models")
	xml_text = models_root.joinpath("softhandV2.xml").read_text(encoding="utf-8")
	source_tag = '<geom contype="0" conaffinity="0" density="0" group="1"/>'
	target_tag = '<geom contype="1" conaffinity="1" density="0" group="1"/>'
	if source_tag not in xml_text:
		raise RuntimeError("Could not find the default geom contact tag in softhandV2.xml")

	xml_text = xml_text.replace(source_tag, target_tag, 1)

	cache_dir = Path(tempfile.gettempdir()) / "robosuite_ros"
	cache_dir.mkdir(parents=True, exist_ok=True)
	cache_mesh_dir = cache_dir / "meshes"
	if not cache_mesh_dir.exists():
		source_mesh_dir = models_root.joinpath("meshes")
		shutil.copytree(source_mesh_dir, cache_mesh_dir)
	xml_path = cache_dir / "softhandV2_contact_enabled.xml"

	if not xml_path.exists() or xml_path.read_text(encoding="utf-8") != xml_text:
		xml_path.write_text(xml_text, encoding="utf-8")

	return xml_path


class ContactEnabledQbHand2MGripper(QbHand2MGripperBase):
	"""QbHand2M gripper variant with collision geoms enabled for grasp tests."""

	def __init__(self, idn: int = 0):
		GripperModel.__init__(self, fname=str(_contact_enabled_gripper_xml_path()), idn=idn)


def _build_world() -> tuple[MujocoWorldBase, ContactEnabledQbHand2MGripper]:
	world = MujocoWorldBase()

	arena = TableArena(table_full_size=(0.4, 0.4, 0.05), table_offset=(0, 0, 1.1), has_legs=False)
	world.merge(arena)

	gripper = ContactEnabledQbHand2MGripper()
	gripper_body = ET.Element("body", name="gripper_base")
	gripper_body.set("pos", "0 -0.15 1.30")
	gripper_body.set("quat", "-1 1 0 0")
	gripper_body.append(new_joint(name="gripper_z_joint", type="slide", axis="0 1 0", damping="50"))
	world.worldbody.append(gripper_body)
	world.merge(gripper, merge_body="gripper_base")
	world.actuator.append(new_actuator(joint="gripper_z_joint", act_type="position", name="gripper_z", kp="500"))

	grasp_box = BoxObject(
		name="box",
		size=[0.02, 0.02, 0.02],
		rgba=[1, 0, 0, 1],
		friction=[1, 0.005, 0.0001],
	).get_obj()
	grasp_box.set("pos", "0 0 1.165")
	world.worldbody.append(grasp_box)

	pedestal = BoxObject(
		name="pedestal",
		size=[0.03, 0.03, 0.01],
		rgba=[0.25, 0.25, 0.25, 1],
		friction=[1, 0.01, 0.0001],
	).get_obj()
	pedestal.set("pos", "0 0 1.135")
	world.worldbody.append(pedestal)

	x_ref = BoxObject(name="x_ref", size=[0.01, 0.01, 0.01], rgba=[0, 1, 0, 1], obj_type="visual", joints=None).get_obj()
	x_ref.set("pos", "0.2 0 1.105")
	world.worldbody.append(x_ref)

	y_ref = BoxObject(name="y_ref", size=[0.01, 0.01, 0.01], rgba=[0, 0, 1, 1], obj_type="visual", joints=None).get_obj()
	y_ref.set("pos", "0 0.2 1.105")
	world.worldbody.append(y_ref)

	return world, gripper


def _print_contacts(sim: MjSim) -> None:
	for contact in sim.data.contact[0 : sim.data.ncon]:
		geom_name1 = sim.model.geom_id2name(contact.geom1)
		geom_name2 = sim.model.geom_id2name(contact.geom2)
		if geom_name1 == "floor" and geom_name2 == "floor":
			continue
		print(f"contact: {geom_name1} <-> {geom_name2}")


def main() -> None:
	use_renderer = os.getenv("ROBOSUITE_RENDER", "0") == "1"

	world, gripper = _build_world()
	model = world.get_model(mode="mujoco")

	sim = MjSim(model)
	viewer = OpenCVViewer(sim) if use_renderer else None
	sim.add_render_context(MjRenderContextOffscreen(sim, device_id=-1))

	sim_state = sim.get_state()
	sim.set_state(sim_state)

	gripper_z_id = sim.model.actuator_name2id("gripper_z")
	gripper_z_high = 0.18
	gripper_z_low = 0.18
	gripper_jaw_ids = [sim.model.actuator_name2id(name) for name in gripper.actuators]
	# QbHand actuators are position-controlled with ctrlrange [0, 1].
	gripper_open = [0.0, 0.0]
	gripper_closed = [1.0, 1.0]
	slider_qvel_indexes = [sim.model.get_joint_qvel_addr("gripper_z_joint")]

	phases = [
		("approach_open", 500, gripper_z_high, gripper_open),
		("descend_open", 100, gripper_z_low, gripper_open),
		("close", 200, gripper_z_low, gripper_closed),
		("lift_closed", 300, -0.2, gripper_closed),
		("descend_closed", 100, gripper_z_low, gripper_closed),
        ("open", 200, gripper_z_low, gripper_open),
		("lift_open", 300, -0.2, gripper_open),
	]

	step = 0
	try:
		for phase_name, duration, z_target, jaw_target in phases:
			print(f"phase: {phase_name}")
			for _ in range(duration):
				sim.data.ctrl[gripper_z_id] = z_target
				sim.data.ctrl[gripper_jaw_ids] = jaw_target
				sim.step()
				sim.data.qfrc_applied[slider_qvel_indexes] = sim.data.qfrc_bias[slider_qvel_indexes]

				if step % 100 == 0:
					print(f"step: {step}")
					_print_contacts(sim)

				if viewer is not None:
					viewer.render()

				step += 1

		print("test_pick completed")
	finally:
		if viewer is not None:
			del viewer


if __name__ == "__main__":
	main()
