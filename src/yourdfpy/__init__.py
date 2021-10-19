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

import os
import random
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List

import trimesh
import trimesh.transformations as tra

# import xml.etree.ElementTree as ET
from lxml import etree

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
    scale: Optional[float] = None

@dataclass
class Geometry:
    box: Optional[Box] = None
    cylinder: Optional[Cylinder] = None
    sphere: Optional[Sphere] = None
    mesh: Optional[Mesh] = None

@dataclass
class Color:
    rgba: np.ndarray

@dataclass
class Texture:
    filename: str

@dataclass
class Material:
    color: Optional[Color] = None
    texture: Optional[Texture] = None

@dataclass
class Visual:
    name: Optional[str] = None
    origin: Optional[np.ndarray] = None
    geometry: Optional[Geometry] = None  # That's not really optional according to ROS
    material: Optional[Material] = None

@dataclass
class Collision:
    name: str
    origin: Optional[np.ndarray] = None
    geometry: Geometry = None

@dataclass
class Inertial:
    origin: Optional[np.ndarray] = None
    mass: Optional[float] = None
    inertia: Optional[np.ndarray] = None

@dataclass
class Link:
    name: str
    inertial: Optional[Inertial] = None
    visuals: List[Visual] = field(default_factory=list)
    collisions: List[Collision] = field(default_factory=list)

@dataclass
class Dynamics:
    damping: Optional[float] = None
    friction: Optional[float] = None

@dataclass
class Limit:
    effort: Optional[float] = None
    velocity: Optional[float] = None
    lower: Optional[float] = None
    upper: Optional[float] = None

@dataclass
class Joint:
    name: str
    type: str = None
    parent: str = None
    child: str = None
    origin: np.ndarray = None
    axis: np.ndarray = None
    dynamics: Optional[Dynamics] = None
    limit: Optional[Limit] = None

@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    transmission: List[str] = field(default_factory=list)
    gazebo: List[str] = field(default_factory=list)

class URDF:
    def __init__(self, robot=None, mesh_dir=None):
        self.robot = robot
        self.mesh_dir = mesh_dir

    def _parse_box(xml_element):
        return Box(size=np.array(xml_element.attrib['size'].split()))
    
    def _write_box(self, xml_parent, box):
        etree.SubElement(
            xml_parent,
            'box',
            attrib={'size': ' '.join(map(str, box.size))}
        )
    
    def _parse_cylinder(xml_element):
        return Cylinder(radius=float(xml_element.attrib['radius']), length=float(xml_element.attrib['length']))
    
    def _write_cylinder(self, xml_parent, cylinder):
        etree.SubElement(
            xml_parent,
            'cylinder',
            attrib={
                'radius': str(cylinder.radius),
                'length': str(cylinder.length)
            }
        )
    
    def _parse_sphere(xml_element):
        return Sphere(radius=float(xml_element.attrib['radius']))
    
    def _write_sphere(self, xml_parent, sphere):
        etree.SubElement(
            xml_parent,
            'sphere',
            attrib={
                'radius': str(sphere.radius)
            }
        )
    
    def _parse_mesh(xml_element):
        return Mesh(filename=xml_element.get('filename'), scale=float(xml_element.get('scale')))
    
    def _write_mesh(self, xml_parent, mesh):
        attrib = {'filename': mesh.filename}
        if mesh.scale is not None:
            attrib['scale'] = " ".join([str(mesh.scale)]*3)
        
        etree.SubElement(
            xml_parent,
            'mesh',
            attrib=attrib
        )
    
    def _parse_geometry(xml_element):
        geometry = Geometry()
        if xml_element[0].tag == 'box':
            geometry.box = _parse_box(xml_element[0])
        elif xml_element[0].tag == 'cylinder':
            geometry.cylinder = _parse_cylinder(xml_element[0])
        elif xml_element[0].tag == 'sphere':
            geometry.sphere = _parse_sphere(xml_element[0])
        elif xml_element[0].tag == 'mesh':
            geometry.mesh = _parse_mesh(xml_element[0])
        else:
            raise ValueError(f"Unknown tag: {xml_element[0].tag}")
        
        return geometry

    def _write_geometry(self, xml_parent, geometry):
        if geometry is None:
            return
        
        xml_element = etree.SubElement(
            xml_parent,
            'geometry'
        )
        if geometry.box is not None:
            self._write_box(xml_element, geometry.box)
        elif geometry.cylinder is not None:
            self._write_cylinder(xml_element, geometry.cylinder)
        elif geometry.sphere is not None:
            self._write_sphere(xml_element, geometry.sphere)
        elif geometry.mesh is not None:
            self._write_mesh(xml_element, geometry.mesh)
    
    def _parse_origin(xml_element):
        if xml_element is None:
            return None
        
        xyz = xml_element.get('xyz', default='0 0 0')
        rpy = xml_element.get('rpy', default='0 0 0')

        return tra.compose_matrix(translate=np.array(list(map(float, xyz.split()))), angles=np.array(list(map(float, rpy.split()))))
    
    def _write_origin(self, xml_parent, origin):
        if origin is None:
            return

        etree.SubElement(
            xml_parent,
            'origin',
            attrib={
                'xyz': ' '.join(map(str, tra.translation_from_matrix(origin))),
                'rpy': ' '.join(map(str, tra.euler_from_matrix(origin))),
            }
        )

    def _parse_color(xml_element):
        if xml_element is None:
            return None

        rgba = xml_element.get('rgba', default='1 1 1 1')

        return Color(rgba=np.array(list(map(float, rgba.split()))))

    def _write_color(self, xml_parent, color):
        if color is None:
            return
        
        etree.SubElement(
            xml_parent,
            'color',
            attrib={
                'rgba': ' '.join(map(str, color.rgba))
            }
        )

    def _parse_texture(xml_element):
        if xml_element is None:
            return None

        return Texture(filename=xml_element.get('filename', default=None))
    
    def _write_texture(self, xml_parent, texture):
        if texture is None:
            return
        
        etree.SubElement(
            xml_parent,
            'texture',
            attrib={
                'filename': texture.filename
            }
        )
    
    def _parse_material(xml_element):
        if xml_element is None:
            return None
        
        material = Material()
        material.color = _parse_color(xml_element.find('color'))
        material.texture = _parse_texture(xml_element.find('texture'))

        return material

    def _write_material(self, xml_parent, material):
        if material is None:
            return
        
        xml_element = etree.SubElement(xml_parent, 'material')

        self._write_color(xml_element, material.color)
        self._write_texture(xml_element, material.texture)
    
    def _parse_visual(xml_element):
        visual = Visual(name=xml_element.get('name'))

        visual.geometry = _parse_geometry(xml_element.find('geometry'))
        visual.origin = _parse_origin(xml_element.find('origin'))
        visual.material = _parse_material(xml_element.find('material'))

        return visual

    def _write_visual(self, xml_parent, visual):
        xml_element = etree.SubElement(xml_parent, 'visual')

        self._write_geometry(xml_element, visual.geometry)
        self._write_origin(xml_element, visual.origin)
        self._write_material(xml_element, visual.material)

    def _parse_collision(xml_element):
        collision = Collision(name=xml_element.get('name'))

        collision.geometry = _parse_geometry(xml_element.find('geometry'))
        collision.origin = _parse_origin(xml_element.find('origin'))

        return collision
    
    def _write_collision(self, xml_parent, collision):
        xml_element = etree.SubElement(xml_parent, 'collision')

        self._write_geometry(xml_element, collision.geometry)
        self._write_origin(xml_element, collision.origin)

    def _parse_inertia(xml_element):
        if xml_element is None:
            return None
        
        x = xml_element

        return np.array([
            [x.get('ixx', default=1.0), x.get('ixy', default=0.0), x.get('ixz', default=0.0)],
            [x.get('ixy', default=0.0), x.get('iyy', default=1.0), x.get('iyz', default=0.0)],
            [x.get('ixz', default=0.0), x.get('iyz', default=0.0), x.get('izz', default=1.0)],
        ])
    
    def _write_inertia(self, xml_parent, inertia):
        if inertia is None:
            return None

        etree.SubElement(
            xml_parent,
            'inertia',
            attrib={
                'ixx': str(inertia[0, 0]),
                'ixy': str(inertia[0, 1]),
                'ixz': str(inertia[0, 2]),
                'iyy': str(inertia[1, 1]),
                'iyz': str(inertia[1, 2]),
                'izz': str(inertia[2, 2]),
            }
        )

    def _parse_mass(xml_element):
        if xml_element is None:
            return None
        
        return xml_element.get('value', default=1.0)
    
    def _write_mass(self, xml_parent, mass):
        if mass is None:
            return
        
        etree.SubElement(
            xml_parent,
            'mass',
            attrib={
                'value': str(mass),
            }
        )
    
    def _parse_inertial(xml_element):
        if xml_element is None:
            return None

        inertial = Inertial()
        inertial.origin = _parse_origin(xml_element.find('origin'))
        inertial.inertia = _parse_inertia(xml_element.find('inertia'))
        inertial.mass = _parse_mass(xml_element.find('mass'))
        
        return inertial
    
    def _write_inertial(self, xml_parent, inertial):
        if inertial is None:
            return
        
        xml_element = etree.SubElement(xml_parent, 'inertial')

        self._write_origin(xml_element, inertial.origin)
        self._write_mass(xml_element, inertial.mass)
        self._write_inertia(xml_element, inertial.inertia)
    
    def _parse_link(xml_element):
        link = Link(name=xml_element.attrib['name'])

        link.inertial = _parse_inertial(xml_element.find('inertial'))

        for v in xml_element.findall('visual'):
            link.visuals.append(_parse_visual(v))

        for c in xml_element.findall('collision'):
            link.collisions.append(_parse_collision(c))

        return link

    def _write_link(self, xml_parent, link):
        xml_element = etree.SubElement(
            xml_parent,
            'link',
            attrib={
                'name': link.name,
            },
        )

        self._write_inertial(xml_element, link.inertial)
        for visual in link.visuals:
            self._write_visual(xml_element, visual)
        for collision in link.collisions:
            self._write_collision(xml_element, collision)
    
    def _parse_axis(xml_element):
        if xml_element is None:
            return np.array([1.0, 0, 0])
        
        xyz = xml_element.get('xyz', '1 0 0')
        return np.array(list(map(float, xyz.split())))
    
    def _write_axis(self, xml_parent, axis):
        if axis is None:
            return

        etree.SubElement(
            xml_parent,
            'axis',
            attrib = {'xyz': ' '.join(map(str, axis))}
        )

    def _parse_limit(xml_element):
        if xml_element is None:
            return None
        
        limit = Limit()
        limit.effort = xml_element.get('effort', default=None)
        limit.velocity = xml_element.get('velocity', default=None)
        limit.lower = xml_element.get('lower', default=None)
        limit.upper = xml_element.get('upper', default=None)

        return limit
    
    def _write_limit(self, xml_parent, limit):
        if limit is None:
            return

        attrib = {}
        if limit.effort is not None:
            attrib['effort'] = str(limit.effort)
        if limit.velocity is not None:
            attrib['velocity'] = str(limit.velocity)
        if limit.lower is not None:
            attrib['lower'] = str(limit.lower)
        if limit.upper is not None:
            attrib['upper'] = str(limit.upper)
        
        etree.SubElement(
            xml_parent,
            'limit',
            attrib=attrib,
        )
    
    def _parse_dynamics(xml_element):
        if xml_element is None:
            return None
        
        dynamics = Dynamics()
        dynamics.damping = xml_element.get('damping', default=None)
        dynamics.friction = xml_element.get('friction', default=None)

        return dynamics

    def _write_dynamics(self, xml_parent, dynamics):
        if dynamics is None:
            return

        attrib = {}
        if dynamics.damping is not None:
            attrib['damping'] = str(dynamics.damping)
        if dynamics.friction is not None:
            attrib['friction'] = str(dynamics.friction)
        
        etree.SubElement(
            xml_parent,
            'dynamics',
            attrib=attrib,
        )
    
    def _parse_joint(xml_element):
        joint = Joint(name=xml_element.attrib['name'])
        
        joint.type = xml_element.get('type', default=None)
        joint.parent = xml_element.find('parent').get('link')
        joint.child = xml_element.find('child').get('link')
        joint.origin = _parse_origin(xml_element.find('origin'))
        joint.axis = _parse_axis(xml_element.find('axis'))
        joint.limit = _parse_limit(xml_element.find('limit'))
        joint.dynamics = _parse_dynamics(xml_element.find('dynamics'))

        return joint

    def _write_joint(self, xml_parent, joint):
        xml_element = etree.SubElement(
            xml_parent,
            'joint',
            attrib={
                'name': joint.name,
                'type': joint.type,
            }
        )

        etree.SubElement(
            xml_element,
            'parent',
            attrib={'link': joint.parent}
        )
        etree.SubElement(
            xml_element,
            'child',
            attrib={'link': joint.child}
        )
        self._write_origin(xml_element, joint.origin)
        self._write_axis(xml_element, joint.axis)
        self._write_limit(xml_element, joint.limit)
        self._write_dynamics(xml_element, joint.dynamics)

    def _parse_robot(xml_element):
        robot = Robot(name=xml_element.attrib['name'])

        for l in xml_element.findall('link'):
            robot.links.append(_parse_link(l))
        for j in xml_element.findall('joint'):
            robot.joints.append(_parse_joint(j))
        
        return robot
    
    def _write_robot(self, robot):
        xml_element = etree.Element(
            'robot',
            attrib={
                'name': robot.name
            }
        )
        for link in robot.links:
            self._write_link(xml_element, link)
        for joint in robot.joints:
            self._write_joint(xml_element, joint)

        return xml_element

    def from_xml_file(fname):
        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(fname, parser)

        root = tree.getroot()
        robot = _parse_robot(root)

        return URDF(robot=robot, mesh_dir=os.path.dirname(fname))

    def _determine_base_link(self):
        link_names = [l.name for l in self.robot.links]

        for j in self.robot.joints:
            link_names.remove(j.child)
        
        if len(link_names) == 0:
            # raise Error?
            return None
        
        return random.choice(link_names)

    def _forward_kinematics_joint(self, joint, q=0.0):
        origin = np.eye(4) if joint.origin is None else joint.origin
        
        if joint.type == 'revolute':
            matrix = origin @ tra.rotation_matrix(q, joint.axis)
        elif joint.type == 'prismatic':
            matrix = origin @ tra.translation_matrix(q*joint.axis)
        else:
            matrix = origin

        return matrix

    def update_trimesh_scene(self, trimesh_scene, configuration):
        # TODO: keep track of non-actuated joints
        if len(configuration) != len(self.robot.joints):
            raise ValueError(f"Dimensionality of configuration ({len(configuration)}) doesn't match number of actuated joints ({len(self.robot.joints)}).")
        
        for j, q in zip(self.robot.joints, configuration):
            matrix = self._forward_kinematics_joint(j, q=q)
            
            trimesh_scene.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)

    def _add_visual_to_scene(self, s, v, link_name):
        origin = v.origin if v.origin is not None else np.eye(4)

        if v.geometry is not None:
            if v.geometry.box is not None:
                new_s = trimesh.Scene([trimesh.creation.box(extents=v.geometry.box.size)])
            elif v.geometry.sphere is not None:
                new_s = trimesh.Scene([trimesh.creation.uv_sphere(radius=v.geometry.sphere.radius)])
            elif v.geometry.cylinder is not None:
                new_s = trimesh.Scene([trimesh.creation.cylinder(radius=v.geometry.cylinder.radius, height=v.geometry.cylinder.length)])
            elif v.geometry.mesh is not None:
                print(f'Loading {v.geometry.mesh.filename} from {self.mesh_dir}')
                new_s = trimesh.load(os.path.join(self.mesh_dir, v.geometry.mesh.filename), force='scene')
            
            for name, geom in new_s.geometry.items():
                s.add_geometry(
                    geometry=geom,
                    parent_node_name=link_name,
                    transform=origin @ new_s.graph.get(name)[0],
                )

    def get_scene(self, configuration=None):
        s = trimesh.scene.Scene(base_frame=self._determine_base_link())

        configuration = np.zeros(len(self.robot.joints)) if configuration is None else configuration
        assert(len(configuration) == len(self.robot.joints))

        for j, q in zip(self.robot.joints, configuration):
            matrix = self._forward_kinematics_joint(j, q=q)
            
            s.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)

        for l in self.robot.links:
            s.graph.nodes.add(l.name)
            for v in l.visuals:
                self._add_visual_to_scene(s, v, link_name=l.name)

        
        return s

    def write_xml(self):
        xml_element = self._write_robot(self.robot)
        return etree.ElementTree(xml_element)

    def write_xml_string(self, **kwargs):
        xml_element = self.write_xml()
        return etree.tostring(xml_element, xml_declaration=True, *kwargs)

    def write_xml_file(self, fname):
        xml_element = self.write_xml()
        xml_element.write(fname, xml_declaration=True, pretty_print=True)

        
        
