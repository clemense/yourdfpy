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
