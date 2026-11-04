from io import BytesIO
from copy import deepcopy
import pytest
from lxml import etree
from yourdfpy import URDF


def load(xml):
    return URDF.load(BytesIO(xml), build_scene_graph=False, load_meshes=False)


@pytest.mark.parametrize("reduction", [None, 0.0, 50.0])
def test_transmission_survives_robot_roundtrip(reduction):
    reduction_xml = (
        ""
        if reduction is None
        else "<mechanicalReduction>{}</mechanicalReduction>".format(reduction)
    )
    xml = (
        """<robot name="robot"><link name="base"/>
    <transmission name="drive"><type>transmission_interface/SimpleTransmission</type>
      <joint name="joint"><hardwareInterface>PositionJointInterface</hardwareInterface></joint>
      <actuator name="motor"><hardwareInterface>EffortJointInterface</hardwareInterface>
      """
        + reduction_xml
        + """</actuator></transmission></robot>"""
    ).encode()
    robot = load(xml)
    assert len(robot.robot.transmission) == 1
    transmission = robot.robot.transmission[0]
    assert transmission.type == "transmission_interface/SimpleTransmission"
    assert transmission.joints[0].hardware_interfaces == ["PositionJointInterface"]
    assert transmission.actuators[0].mechanical_reduction == reduction
    assert transmission.actuators[0].hardware_interfaces == ["EffortJointInterface"]
    output = robot.write_xml_string()
    element = etree.fromstring(output).find("transmission")
    assert element is not None
    assert element.findtext("type") == transmission.type
    assert (
        load(output).robot.transmission[0].actuators[0].mechanical_reduction
        == reduction
    )
    assert robot == load(output)
    different = deepcopy(transmission.actuators[0])
    different.mechanical_reduction = 2.0 if reduction is None else None
    assert transmission.actuators[0] != different


def test_robot_without_transmissions_still_roundtrips():
    robot = load(b'<robot name="robot"><link name="base"/></robot>')
    assert robot.robot.transmission == []
    assert etree.fromstring(robot.write_xml_string()).find("transmission") is None


def test_multiple_transmissions_and_interfaces_keep_their_order():
    xml = b"""<robot name="robot"><link name="base"/>
      <transmission name="first"><type>first_type</type>
        <joint name="a"><hardwareInterface>one</hardwareInterface>
          <hardwareInterface>two</hardwareInterface></joint></transmission>
      <transmission name="second"><actuator name="b"/></transmission>
    </robot>"""
    robot = load(xml)
    restored = load(robot.write_xml_string())
    assert [t.name for t in restored.robot.transmission] == ["first", "second"]
    assert restored.robot.transmission[0].joints[0].hardware_interfaces == [
        "one",
        "two",
    ]
    assert restored.robot.transmission[1].type is None
