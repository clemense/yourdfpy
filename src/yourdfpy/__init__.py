import sys

if sys.version_info[:2] >= (3, 8):
    # TODO: Import directly (no need for conditional) when `python_requires = >= 3.8`
    from importlib.metadata import PackageNotFoundError, version  # pragma: no cover
else:
    from importlib_metadata import PackageNotFoundError, version  # pragma: no cover

try:
    # Change here if project is renamed and does not equal the package name
    dist_name = __name__
    __version__ = version(dist_name)
except PackageNotFoundError:  # pragma: no cover
    __version__ = "unknown"
finally:
    del version, PackageNotFoundError


import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List

import trimesh.transformations as tra

import xml.etree.ElementTree as ET

@dataclass
class Sphere:
    radius: float

@dataclass
class Cylinder:
    radius: float
    length: float

@dataclass
class Box:
    size: np.ndarray

@dataclass
class Mesh:
    filename: str
    scale: float

@dataclass
class Geometry:
    box: Optional[Box] = None
    cylinder: Optional[Cylinder] = None
    sphere: Optional[Sphere] = None
    mesh: Optional[Mesh] = None

@dataclass
class Material:
    color: np.ndarray
    texture: str

@dataclass
class Visual:
    name: Optional[str] = None
    origin: Optional[np.ndarray] = None
    geometry: Optional[Geometry] = None  # That's not really optional according to ROS
    material: Optional[Material] = None

@dataclass
class Collision:
    name: str
    origin: np.ndarray
    geometry: Geometry

@dataclass
class Inertial:
    origin: np.ndarray
    mass: float
    inertia: np.ndarray

@dataclass
class Link:
    name: str
    inertial: Optional[Inertial] = None
    visuals: List[Visual] = field(default_factory=list)
    collisions: List[Collision] = field(default_factory=list)

@dataclass
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray

@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    transmission: List[str] = field(default_factory=list)
    gazebo: List[str] = field(default_factory=list)

class URDF:
    def __init__(self, xml_tree):
        root = xml_tree.getroot()
        
        self.robot = self._parse_robot(root)
        

    def _parse_box(self, xml_element):
        return Box(size=np.array(xml_element.attrib['size'].split()))
    
    def _parse_cylinder(self, xml_element):
        return Cylinder(radius=xml_element.attrib['radius'], length=xml_element.attrib['length'])
    
    def _parse_sphere(self, xml_element):
        return Sphere(radius=xml_element.attrib['radius'])
    
    def _parse_mesh(self, xml_element):
        return Mesh(filename=xml_element.get('filename'), scale=xml_element.get('scale'))
    
    def _parse_geometry(self, xml_element):
        geometry = Geometry()
        if xml_element[0].tag == 'box':
            geometry.box = self._parse_box(xml_element[0])
        elif xml_element[0].tag == 'cylinder':
            geometry.cylinder = self._parse_cylinder(xml_element[0])
        elif xml_element[0].tag == 'sphere':
            geometry.sphere = self._parse_sphere(xml_element[0])
        elif xml_element[0].tag == 'mesh':
            geometry.mesh = self._parse_mesh(xml_element[0])
        else:
            raise ValueError(f"Unknown tag: {xml_element[0].tag}")
        
        return geometry

    def _parse_origin(self, xml_element):
        if xml_element is None:
            return None
        
        xyz = xml_element.get('xyz', default='0 0 0')
        rpy = xml_element.get('rpy', default='0 0 0')

        return tra.compose_matrix(translate=np.array(list(map(float, xyz.split()))), angles=np.array(list(map(float, rpy.split()))))
    
    def _parse_visual(self, xml_element):
        visual = Visual(name=xml_element.get('name'))

        visual.geometry = self._parse_geometry(xml_element.find('geometry'))
        visual.origin = self._parse_origin(xml_element.find('origin'))

        return visual

    def _parse_link(self, xml_element):
        link = Link(name=xml_element.attrib['name'])

        for v in xml_element.findall('visual'):
            link.visuals.append(self._parse_visual(v))

        return link

    def _parse_robot(self, xml_element):
        robot = Robot(name=xml_element.attrib['name'])

        for l in xml_element.findall('link'):
            robot.links.append(self._parse_link(l))

        return robot
    
    def from_xml_file(fname):
        tree = ET.parse(fname)

        return URDF(tree)
            

