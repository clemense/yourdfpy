"""Real mesh resolution across adjacent URDF packages."""

import logging
import os
from pathlib import Path

import numpy as np
import pytest

from yourdfpy import urdf


def _write_obj(path, vertices):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = ["v {} {} {}".format(*vertex) for vertex in vertices]
    path.write_text("\n".join(rows + ["f 1 2 3", ""]), encoding="utf-8")
    return path


def _outside_directory(tmp_path, monkeypatch):
    outside = tmp_path / "outside"
    outside.mkdir()
    monkeypatch.chdir(outside)
    return outside


def _world_bounds(scene):
    bounds = []
    for node in scene.graph.nodes_geometry:
        transform, geometry_name = scene.graph.get(node)
        vertices = scene.geometry[geometry_name].vertices
        world = vertices @ transform[:3, :3].T + transform[:3, 3]
        bounds.append(np.stack((world.min(axis=0), world.max(axis=0))))
    return sorted(bounds, key=lambda bound: bound[0, 0])


@pytest.mark.parametrize("relative", [False, True])
def test_package_directory_is_retained_at_ancestor(tmp_path, monkeypatch, relative):
    outside = _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "robot repository"
    urdf_dir = root / "robot_description" / "urdf" / "nested"
    urdf_dir.mkdir(parents=True)
    target = _write_obj(
        root / "hand_description" / "meshes" / "part.obj",
        [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
    )
    directory = os.path.relpath(urdf_dir, outside) if relative else str(urdf_dir)

    result = urdf.filename_handler_magic(
        "package://hand_description/meshes/part.obj", dir=directory
    )

    assert Path(result).resolve() == target.resolve()
    assert Path(result).read_bytes() == target.read_bytes()


@pytest.mark.parametrize("package", ["hand_description", "hand package \u03b1"])
def test_explicit_mesh_root_contains_package_directory(tmp_path, monkeypatch, package):
    _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "packages"
    target = _write_obj(
        root / package / "meshes" / "part.obj",
        [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
    )

    result = urdf.filename_handler_magic(
        "package://{}/meshes/part.obj".format(package), dir=str(root)
    )

    assert Path(result).resolve() == target.resolve()


@pytest.mark.parametrize("force_mesh", [False, True])
def test_urdf_loads_two_sibling_package_meshes(tmp_path, monkeypatch, force_mesh):
    _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "robot repository"
    vertices_a = np.array([(0, 0, 0), (2, 0, 0), (0, 3, 0)], dtype=float)
    vertices_b = np.array([(10, 20, 30), (14, 20, 30), (10, 25, 30)], dtype=float)
    _write_obj(root / "arm_description" / "meshes" / "part.obj", vertices_a)
    _write_obj(root / "hand_description" / "meshes" / "part.obj", vertices_b)
    model_path = root / "robot_description" / "urdf" / "robot.urdf"
    model_path.parent.mkdir(parents=True)
    model_path.write_text(
        """<robot name="two_packages">
  <link name="base">
    <visual name="arm_mesh">
      <origin xyz="1 2 3" />
      <geometry><mesh filename="package://arm_description/meshes/part.obj" />
      </geometry>
    </visual>
  </link>
  <link name="hand">
    <visual name="hand_mesh">
      <origin xyz="0 0 2" />
      <geometry><mesh filename="package://hand_description/meshes/part.obj" />
      </geometry>
    </visual>
  </link>
  <joint name="hand_fixed" type="fixed">
    <parent link="base" /><child link="hand" /><origin xyz="5 0 0" />
  </joint>
</robot>
""",
        encoding="utf-8",
    )

    model = urdf.URDF.load(str(model_path), force_mesh=force_mesh)

    assert len(model.scene.geometry) == 2
    actual = _world_bounds(model.scene)
    for bounds, vertices, translation in zip(
        actual, [vertices_a, vertices_b], [[1, 2, 3], [5, 0, 2]]
    ):
        expected = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
        np.testing.assert_allclose(bounds, expected + translation, rtol=0, atol=1e-12)
    for link, package in zip(
        model.robot.links, ["arm_description", "hand_description"]
    ):
        assert link.visuals[0].geometry.mesh.filename == (
            "package://{}/meshes/part.obj".format(package)
        )


@pytest.mark.parametrize("preferred", ["legacy_relative", "cwd"])
def test_existing_resolution_priority_is_preserved(tmp_path, monkeypatch, preferred):
    outside = _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "repository"
    urdf_dir = root / "robot_description" / "urdf"
    urdf_dir.mkdir(parents=True)
    new_fallback = _write_obj(
        root / "hand_description" / "meshes" / "part.obj",
        [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
    )
    if preferred == "legacy_relative":
        target = urdf_dir / "meshes" / "part.obj"
    else:
        target = outside / "hand_description" / "meshes" / "part.obj"
    _write_obj(target, [(0, 0, 0), (7, 0, 0), (0, 9, 0)])

    result = urdf.filename_handler_magic(
        "package://hand_description/meshes/part.obj", dir=str(urdf_dir)
    )

    assert Path(result).resolve() == target.resolve()
    assert Path(result).read_bytes() != new_fallback.read_bytes()


@pytest.mark.parametrize("directive", ["", "file://"])
def test_nonpackage_mesh_inputs_are_unchanged(tmp_path, monkeypatch, directive):
    _outside_directory(tmp_path, monkeypatch)
    urdf_dir = tmp_path / "model"
    target = _write_obj(
        urdf_dir / "meshes" / "part.obj", [(0, 0, 0), (1, 0, 0), (0, 1, 0)]
    )

    result = urdf.filename_handler_magic(
        directive + "meshes/part.obj", dir=str(urdf_dir)
    )

    assert Path(result).resolve() == target.resolve()


def test_unresolved_package_uri_is_preserved(tmp_path, monkeypatch, caplog):
    _outside_directory(tmp_path, monkeypatch)
    urdf_dir = tmp_path / "repository" / "robot_description" / "urdf"
    urdf_dir.mkdir(parents=True)
    filename = "package://absent_package/meshes/not_present.obj"

    with caplog.at_level(logging.WARNING, logger="yourdfpy.urdf"):
        result = urdf.filename_handler_magic(filename, dir=str(urdf_dir))

    assert result == filename
    assert [record.message for record in caplog.records] == [
        "Unable to resolve filename: {}".format(filename)
    ]


def test_retained_package_prefers_nearest_ancestor(tmp_path, monkeypatch):
    _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "repository"
    urdf_dir = root / "robot_description" / "urdf"
    urdf_dir.mkdir(parents=True)
    target = _write_obj(
        urdf_dir.parent / "hand_description" / "meshes" / "part.obj",
        [(0, 0, 0), (1, 0, 0), (0, 1, 0)],
    )
    _write_obj(
        root / "hand_description" / "meshes" / "part.obj",
        [(0, 0, 0), (2, 0, 0), (0, 3, 0)],
    )

    result = urdf.filename_handler_magic(
        "package://hand_description/meshes/part.obj", dir=str(urdf_dir)
    )

    assert Path(result).resolve() == target.resolve()


def test_same_package_legacy_mesh_layout_still_loads(tmp_path, monkeypatch):
    _outside_directory(tmp_path, monkeypatch)
    root = tmp_path / "robot_description"
    vertices = np.array([(0, 0, 0), (2, 0, 0), (0, 3, 0)], dtype=float)
    _write_obj(root / "meshes" / "part.obj", vertices)
    model_path = root / "urdf" / "robot.urdf"
    model_path.parent.mkdir(parents=True)
    model_path.write_text(
        """<robot name="same_package"><link name="base"><visual name="mesh">
<geometry><mesh filename="package://robot_description/meshes/part.obj" />
</geometry></visual></link></robot>
""",
        encoding="utf-8",
    )

    model = urdf.URDF.load(str(model_path))

    assert len(model.scene.geometry) == 1
    np.testing.assert_allclose(
        _world_bounds(model.scene)[0],
        np.stack((vertices.min(axis=0), vertices.max(axis=0))),
        rtol=0,
        atol=1e-12,
    )
