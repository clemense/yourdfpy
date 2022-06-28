import pytest
import os
import io

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


def test_single_link_urdf():
    urdf_str = """
    <robot name="single_link">
        <link name="link_0">
            <visual>
                <geometry>
                    <sphere radius="1" />
                </geometry>
            </visual>
        </link>
    </robot>
    """
    with io.StringIO(urdf_str) as f:
        urdf_model = urdf.URDF.load(f)

        assert len(urdf_model.scene.graph.to_edgelist()) == 1


def test_material_color():
    urdf_str = """
    <robot name="material_test">
        <link name="link_0">
            <visual>
                <geometry>
                    <sphere radius="1" />
                </geometry>
                <material name="red_material">
                    <color rgba="1.0 0.0 0.0 1.0" />
                </material>
            </visual>
        </link>
    </robot>
    """
    with io.StringIO(urdf_str) as f:
        urdf_model = urdf.URDF.load(f)

        assert urdf_model.robot.links[0].visuals[0].material.name == "red_material"
        assert all(
            urdf_model.robot.links[0].visuals[0].material.color.rgba == [1, 0, 0, 1]
        )


def test_material_mapping():
    urdf_str = """
    <robot name="material_test">
        <link name="link_0">
            <visual>
                <geometry>
                    <sphere radius="1" />
                </geometry>
                <material name="red_material" />
            </visual>
        </link>
        <material name="red_material">
            <color rgba="1.0 0.0 0.0 1.0" />
        </material>
    </robot>
    """
    with io.StringIO(urdf_str) as f:
        urdf_model = urdf.URDF.load(f)

        assert urdf_model.robot.links[0].visuals[0].material.name == "red_material"
        assert all(urdf_model._material_map["red_material"].color.rgba == [1, 0, 0, 1])


def test_geometric_primitives():
    urdf_str = """
    <robot name="primitives_test">
        <link name="link_0">
            <visual>
                <geometry>
                    <sphere radius="11" />
                </geometry>
            </visual>
            <visual>
                <geometry>
                    <box size="1 2 3" />
                </geometry>
            </visual>
            <visual>
                <geometry>
                    <cylinder radius="11" length="4"/>
                </geometry>
            </visual>
        </link>
        <material name="red_material">
            <color rgba="1.0 0.0 0.0 1.0" />
        </material>
    </robot>
    """
    with io.StringIO(urdf_str) as f:
        urdf_model = urdf.URDF.load(f)

        assert urdf_model.link_map["link_0"].visuals[0].geometry.sphere.radius == 11
        assert all(
            urdf_model.link_map["link_0"].visuals[1].geometry.box.size == [1, 2, 3]
        )
        assert urdf_model.link_map["link_0"].visuals[2].geometry.cylinder.radius == 11
        assert urdf_model.link_map["link_0"].visuals[2].geometry.cylinder.length == 4
