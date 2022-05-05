import pytest
import os

from yourdfpy import urdf

__author__ = "Clemens Eppner"
__copyright__ = "Clemens Eppner"
__license__ = "MIT"


DIR_CURRENT = os.path.dirname(os.path.abspath(os.path.expanduser(__file__)))
DIR_MODELS = os.path.abspath(os.path.join(DIR_CURRENT, "models"))


def _create_robot():
    r = urdf.Robot(name="test_robot")
    return r


def test_robot():
    r = _create_robot()
    assert r.name == "test_robot"


def test_validate():
    r = _create_robot()
    xml = urdf.URDF(robot=r)
    assert xml.validate()


def test_mimic_joint():
    urdf_fname = os.path.join(DIR_MODELS, "franka", "franka.urdf")
    urdf_model = urdf.URDF.load(urdf_fname)

    assert True


def test_equality():
    urdf_fname = os.path.join(DIR_MODELS, "franka", "franka.urdf")
    urdf_model_0 = urdf.URDF.load(urdf_fname)

    urdf_model_1 = urdf.URDF.load(urdf_fname)

    assert urdf_model_0 == urdf_model_1


def test_equality_different_link_order():
    robot_0 = _create_robot()
    robot_0.links.append(urdf.Link(name="link_0"))
    robot_0.links.append(urdf.Link(name="link_1"))

    robot_1 = _create_robot()
    robot_1.links.append(urdf.Link(name="link_1"))
    robot_1.links.append(urdf.Link(name="link_0"))

    assert robot_0 == robot_1
