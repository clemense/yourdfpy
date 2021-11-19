import pytest

from yourdfpy import urdf

__author__ = "Clemens Eppner"
__copyright__ = "Clemens Eppner"
__license__ = "MIT"


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
