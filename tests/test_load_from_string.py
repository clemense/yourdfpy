"""Tests for loading robot descriptions without a temporary URDF file."""

import numpy as np
import pytest

from yourdfpy import URDF

ROBOT_XML = """<?xml version="1.0" encoding="UTF-8"?>
<robot name="robot_é">
  <link name="base"/>
  <link name="tool"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="tool"/>
    <origin xyz="1 2 3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>
"""


@pytest.mark.parametrize("as_bytes", [False, True])
def test_load_from_string_forward_kinematics(as_bytes):
    """Load text or bytes and evaluate an articulated robot's transform."""
    xml = ROBOT_XML.encode("utf-8") if as_bytes else ROBOT_XML
    model = URDF.load_from_string(xml, load_meshes=False)

    assert model.robot.name == "robot_é"
    assert model.actuated_joint_names == ["hinge"]
    model.update_cfg([np.pi / 2])
    np.testing.assert_allclose(
        model.get_transform("tool", "base"),
        [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]],
        atol=1e-14,
    )


def test_load_from_string_respects_byte_encoding():
    """Honor the declared encoding when the caller supplies XML bytes."""
    xml = ROBOT_XML.replace("UTF-8", "ISO-8859-1").encode("iso-8859-1")
    model = URDF.load_from_string(xml, load_meshes=False)
    assert model.robot.name == "robot_é"


def test_load_from_string_without_scene_graph():
    """Pass loading options through without constructing a visual scene."""
    model = URDF.load_from_string(ROBOT_XML, build_scene_graph=False, load_meshes=False)
    assert model.scene is None
    assert model.num_actuated_joints == 1


@pytest.mark.parametrize("force_mesh", [False, True])
def test_load_from_string_relative_mesh(tmp_path, monkeypatch, force_mesh):
    """Resolve real mesh files against mesh_dir, independent of cwd."""
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "triangle.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    xml = """<robot name="mesh_robot"><link name="base"><visual>
      <geometry><mesh filename="triangle.obj"/></geometry>
    </visual></link></robot>"""
    model = URDF.load_from_string(xml, mesh_dir=str(mesh_dir), force_mesh=force_mesh)
    assert len(model.scene.geometry) == 1
    np.testing.assert_allclose(model.scene.bounds, [[0, 0, 0], [1, 1, 0]])
