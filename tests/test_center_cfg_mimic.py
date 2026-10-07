"""Center configurations contain independent coordinates, not mimic values."""

import math
from itertools import permutations
from pathlib import Path

import numpy as np
import pytest

from yourdfpy import urdf


def _joint(name, kind="revolute", limits=(-0.5, 1.5), mimic=None):
    return urdf.Joint(
        name=name,
        type=kind,
        parent="base",
        child=name + "_link",
        axis=np.array([0.0, 0.0, 1.0]),
        limit=urdf.Limit(lower=limits[0], upper=limits[1]),
        mimic=mimic,
    )


def _model(joints):
    return urdf.URDF(
        urdf.Robot(
            name="center_configuration",
            links=[urdf.Link(name="base")] + [urdf.Link(name=j.child) for j in joints],
            joints=joints,
        ),
        build_collision_scene_graph=True,
        load_meshes=False,
        load_collision_meshes=False,
    )


def _motion(kind, q):
    """Independent closed-form transform about/along the chosen z axis."""
    result = np.eye(4)
    if kind == "prismatic":
        result[2, 3] = q
    elif kind != "fixed":
        c, s = math.cos(q), math.sin(q)
        result[:2, :2] = [[c, -s], [s, c]]
    return result


@pytest.mark.parametrize("kind", ["revolute", "prismatic"])
def test_center_cfg_keeps_independent_coordinates_for_every_joint_order(kind):
    source = _joint("source", kind, (-0.5, 1.5))
    other = _joint("other", kind, (-2.0, 6.0))
    mimic = _joint(
        "mimic", kind, mimic=urdf.Mimic("source", multiplier=-2.0, offset=0.25)
    )
    fixed = _joint("fixed", "fixed")
    values = {"source": 0.5, "other": 2.0}
    for ordered in permutations([source, other, mimic, fixed]):
        model = _model(list(ordered))
        expected = np.array([values[j.name] for j in model.actuated_joints])
        center = model.center_cfg
        assert center.shape == (model.num_dofs,) == (2,)
        np.testing.assert_array_equal(center, expected)
        np.testing.assert_array_equal(model.cfg, [0.0, 0.0])
        model.update_cfg(center)
        np.testing.assert_array_equal(model.cfg, expected)
        for name, q in {**values, "mimic": -0.75}.items():
            for collision in [False, True]:
                np.testing.assert_allclose(
                    model.get_transform(name + "_link", collision_geometry=collision),
                    _motion(kind, q),
                    rtol=0.0,
                    atol=2e-15,
                )


@pytest.mark.parametrize("kind", ["revolute", "prismatic"])
def test_center_cfg_does_not_overwrite_later_actuated_joint(kind):
    # The mimic's robot index is a valid but unrelated independent cfg index.
    model = _model(
        [
            _joint("source", kind, (-0.5, 1.5)),
            _joint("mimic", kind, mimic=urdf.Mimic("source", 3.0, -0.125)),
            _joint("other", kind, (-2.0, 6.0)),
        ]
    )
    np.testing.assert_array_equal(model.center_cfg, [0.5, 2.0])
    model.update_cfg(model.center_cfg)
    np.testing.assert_allclose(
        model.get_transform("mimic_link"), _motion(kind, 1.375), rtol=0.0, atol=2e-15
    )
    np.testing.assert_allclose(
        model.get_transform("other_link"), _motion(kind, 2.0), rtol=0.0, atol=2e-15
    )


def test_center_cfg_multiple_mimics_share_one_independent_coordinate():
    model = _model(
        [
            _joint("source", limits=(-1.0, 2.0)),
            _joint("mimic_1", mimic=urdf.Mimic("source", -1.5, 0.125)),
            _joint("mimic_2", mimic=urdf.Mimic("source", 0.0, -0.375)),
        ]
    )
    np.testing.assert_array_equal(model.center_cfg, [0.5])
    model.update_cfg(model.center_cfg)
    for name, q in [("source", 0.5), ("mimic_1", -0.625), ("mimic_2", -0.375)]:
        np.testing.assert_allclose(
            model.get_transform(name + "_link"),
            _motion("revolute", q),
            rtol=0.0,
            atol=2e-15,
        )


def test_center_cfg_franka_mimic_uses_the_existing_actuated_configuration():
    path = Path(__file__).parent / "models" / "franka" / "franka.urdf"
    model = urdf.URDF.load(path.as_posix(), load_meshes=False)
    expected = np.array(
        [
            j.limit.lower + 0.5 * (j.limit.upper - j.limit.lower)
            if j.limit is not None
            else 0.0
            for j in model.actuated_joints
        ]
    )
    np.testing.assert_array_equal(model.center_cfg, expected)
    assert model.center_cfg.shape == (model.num_dofs,)
    model.update_cfg(model.center_cfg)
    np.testing.assert_array_equal(model.cfg, expected)


def test_center_cfg_controls_without_mimics_and_without_actuation():
    model = _model(
        [
            _joint("unlimited", limits=(-1.0, 1.0)),
            _joint("continuous", "continuous"),
            _joint("fixed", "fixed"),
        ]
    )
    model.robot.joints[0].limit = None
    np.testing.assert_array_equal(model.center_cfg, [0.0, 0.0])
    empty = _model([_joint("fixed", "fixed")])
    assert empty.center_cfg.shape == (0,)
    assert empty.center_cfg.dtype == np.float64
    empty.update_cfg(empty.center_cfg)
