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
import copy
import random
import logging
import numpy as np
from dataclasses import dataclass, field, is_dataclass
from typing import Optional, Union, List
from functools import partial

import trimesh
import trimesh.transformations as tra

# import xml.etree.ElementTree as ET
from lxml import etree

_logger = logging.getLogger(__name__)


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
    scale: Optional[Union[float, np.ndarray]] = None


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


class URDFError(Exception):
    """General URDF exception."""

    def __init__(self, msg):
        super(URDFError, self).__init__()
        self.msg = msg

    def __str__(self):
        return type(self).__name__ + ": " + self.msg

    def __repr__(self):
        return type(self).__name__ + '("' + self.msg + '")'


class URDFIncompleteError(URDFError):
    """Raised when needed data for an object isn't there."""

    pass


class URDFBrokenRefError(URDFError):
    """Raised when a referenced object is not found in the scope."""

    pass


class URDFMalformedError(URDFError):
    """Raised when data is found to be corrupted in some way."""

    pass


class URDFUnsupportedError(URDFError):
    """Raised when some unexpectedly unsupported feature is found."""

    pass


class URDFSaveValidationError(URDFError):
    """Raised when XML validation fails when saving."""

    pass


def _str2float(s):
    """Cast string to float if it is not None. Otherwise return None.

    Args:
        s (str): String to convert or None.

    Returns:
        str or NoneType: The converted string or None.
    """
    return float(s) if s is not None else None


def filename_handler_ignore_directive(fname):
    return fname.split(f":{os.path.sep}{os.path.sep}")[-1]


def filename_handler_ignore_directive_package(fname):
    if fname.startswith("package:"):
        return os.path.sep.join(
            fname.split(f":{os.path.sep}{os.path.sep}")[-1].split(os.path.sep)[1:]
        )
    return filename_handler_ignore_directive(fname)


def filename_handler_relative(fname, mesh_dir):
    return os.path.join(mesh_dir, filename_handler_ignore_directive_package(fname))


def filename_handler_relative_to_urdf_file(fname, urdf_fname):
    return os.path.join(
        os.path.dirname(urdf_fname), filename_handler_ignore_directive(fname)
    )


def filename_handler_magic(fname, mesh_dir):
    for fn in [partial(filename_handler_relative, mesh_dir=mesh_dir)]:
        candidate_fname = fn(fname=fname)
        if os.path.isfile(candidate_fname):
            return candidate_fname
    print(f"Unable to resolve filename: {fname}")
    return fname


class URDF:
    def __init__(
        self,
        xml_root=None,
        robot=None,
        generate_scene_graph=True,
        load_meshes=True,
        filename_handler=None,
        mesh_dir="",
    ):
        assert bool(xml_root is None) ^ bool(robot is None)

        if filename_handler is None:
            self._filename_handler = partial(filename_handler_magic, mesh_dir=mesh_dir)
        else:
            self._filename_handler = filename_handler

        if xml_root is None:
            self.robot = robot
        else:
            self._xml_root = xml_root
            self.robot = URDF._parse_robot(xml_root)

        self._create_maps()

        self.num_actuated_joints = len(
            [j for j in self.robot.joints if j.type != "fixed"]
        )

        self.errors = []
        self.maskedErrors = []

        if generate_scene_graph:
            self._scene = self._create_scene(load_geometry=load_meshes)
            # if load_meshes:
            #     self._load_meshes()
        else:
            self._scene = None

    @property
    def scene(self):
        return self._scene

    @property
    def link_map(self):
        return self._link_map

    @property
    def joint_map(self):
        return self._joint_map

    def show(self, collision_geometry=False):
        if collision_geometry:
            self.scene.show()
        else:
            self.scene.show()

    def _generate_scene_graph(self):
        pass

    def _load_meshes(self):
        pass

    def _create_maps(self):
        self._joint_map = {}
        for j in self.robot.joints:
            self._joint_map[j.name] = j

        self._link_map = {}
        for l in self.robot.links:
            self._link_map[l.name] = l

    def _parse_box(xml_element):
        return Box(size=np.array(xml_element.attrib["size"].split()))

    def _write_box(self, xml_parent, box):
        etree.SubElement(
            xml_parent, "box", attrib={"size": " ".join(map(str, box.size))}
        )

    def _parse_cylinder(xml_element):
        return Cylinder(
            radius=float(xml_element.attrib["radius"]),
            length=float(xml_element.attrib["length"]),
        )

    def _write_cylinder(self, xml_parent, cylinder):
        etree.SubElement(
            xml_parent,
            "cylinder",
            attrib={"radius": str(cylinder.radius), "length": str(cylinder.length)},
        )

    def _parse_sphere(xml_element):
        return Sphere(radius=float(xml_element.attrib["radius"]))

    def _write_sphere(self, xml_parent, sphere):
        etree.SubElement(xml_parent, "sphere", attrib={"radius": str(sphere.radius)})

    def _parse_scale(xml_element):
        if "scale" in xml_element.attrib:
            s = xml_element.get("scale").split()
            if len(s) == 0:
                return None
            elif len(s) == 1:
                return float(s[0])
            else:
                return np.array(list(map(float, s)))
        return None

    def _parse_mesh(xml_element):
        return Mesh(
            filename=xml_element.get("filename"), scale=URDF._parse_scale(xml_element)
        )

    def _write_mesh(self, xml_parent, mesh):
        attrib = {"filename": mesh.filename}
        if mesh.scale is not None:
            if isinstance(mesh.scale, float) or isinstance(mesh.scale, int):
                attrib["scale"] = " ".join([str(mesh.scale)] * 3)
            else:
                attrib["scale"] = " ".join(map(str, mesh.scale))

        etree.SubElement(xml_parent, "mesh", attrib=attrib)

    def _parse_geometry(xml_element):
        geometry = Geometry()
        if xml_element[0].tag == "box":
            geometry.box = URDF._parse_box(xml_element[0])
        elif xml_element[0].tag == "cylinder":
            geometry.cylinder = URDF._parse_cylinder(xml_element[0])
        elif xml_element[0].tag == "sphere":
            geometry.sphere = URDF._parse_sphere(xml_element[0])
        elif xml_element[0].tag == "mesh":
            geometry.mesh = URDF._parse_mesh(xml_element[0])
        else:
            raise ValueError(f"Unknown tag: {xml_element[0].tag}")

        return geometry

    def _write_geometry(self, xml_parent, geometry):
        if geometry is None:
            return

        xml_element = etree.SubElement(xml_parent, "geometry")
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

        xyz = xml_element.get("xyz", default="0 0 0")
        rpy = xml_element.get("rpy", default="0 0 0")

        return tra.compose_matrix(
            translate=np.array(list(map(float, xyz.split()))),
            angles=np.array(list(map(float, rpy.split()))),
        )

    def _write_origin(self, xml_parent, origin):
        if origin is None:
            return

        etree.SubElement(
            xml_parent,
            "origin",
            attrib={
                "xyz": " ".join(map(str, tra.translation_from_matrix(origin))),
                "rpy": " ".join(map(str, tra.euler_from_matrix(origin))),
            },
        )

    def _parse_color(xml_element):
        if xml_element is None:
            return None

        rgba = xml_element.get("rgba", default="1 1 1 1")

        return Color(rgba=np.array(list(map(float, rgba.split()))))

    def _write_color(self, xml_parent, color):
        if color is None:
            return

        etree.SubElement(
            xml_parent, "color", attrib={"rgba": " ".join(map(str, color.rgba))}
        )

    def _parse_texture(xml_element):
        if xml_element is None:
            return None

        return Texture(filename=xml_element.get("filename", default=None))

    def _write_texture(self, xml_parent, texture):
        if texture is None:
            return

        etree.SubElement(xml_parent, "texture", attrib={"filename": texture.filename})

    def _parse_material(xml_element):
        if xml_element is None:
            return None

        material = Material()
        material.color = URDF._parse_color(xml_element.find("color"))
        material.texture = URDF._parse_texture(xml_element.find("texture"))

        return material

    def _write_material(self, xml_parent, material):
        if material is None:
            return

        xml_element = etree.SubElement(xml_parent, "material")

        self._write_color(xml_element, material.color)
        self._write_texture(xml_element, material.texture)

    def _parse_visual(xml_element):
        visual = Visual(name=xml_element.get("name"))

        visual.geometry = URDF._parse_geometry(xml_element.find("geometry"))
        visual.origin = URDF._parse_origin(xml_element.find("origin"))
        visual.material = URDF._parse_material(xml_element.find("material"))

        return visual

    def _write_visual(self, xml_parent, visual):
        xml_element = etree.SubElement(xml_parent, "visual")

        self._write_geometry(xml_element, visual.geometry)
        self._write_origin(xml_element, visual.origin)
        self._write_material(xml_element, visual.material)

    def _parse_collision(xml_element):
        collision = Collision(name=xml_element.get("name"))

        collision.geometry = URDF._parse_geometry(xml_element.find("geometry"))
        collision.origin = URDF._parse_origin(xml_element.find("origin"))

        return collision

    def _write_collision(self, xml_parent, collision):
        xml_element = etree.SubElement(xml_parent, "collision")

        self._write_geometry(xml_element, collision.geometry)
        self._write_origin(xml_element, collision.origin)

    def _parse_inertia(xml_element):
        if xml_element is None:
            return None

        x = xml_element

        return np.array(
            [
                [
                    x.get("ixx", default=1.0),
                    x.get("ixy", default=0.0),
                    x.get("ixz", default=0.0),
                ],
                [
                    x.get("ixy", default=0.0),
                    x.get("iyy", default=1.0),
                    x.get("iyz", default=0.0),
                ],
                [
                    x.get("ixz", default=0.0),
                    x.get("iyz", default=0.0),
                    x.get("izz", default=1.0),
                ],
            ]
        )

    def _write_inertia(self, xml_parent, inertia):
        if inertia is None:
            return None

        etree.SubElement(
            xml_parent,
            "inertia",
            attrib={
                "ixx": str(inertia[0, 0]),
                "ixy": str(inertia[0, 1]),
                "ixz": str(inertia[0, 2]),
                "iyy": str(inertia[1, 1]),
                "iyz": str(inertia[1, 2]),
                "izz": str(inertia[2, 2]),
            },
        )

    def _parse_mass(xml_element):
        if xml_element is None:
            return None

        return xml_element.get("value", default=1.0)

    def _write_mass(self, xml_parent, mass):
        if mass is None:
            return

        etree.SubElement(
            xml_parent,
            "mass",
            attrib={
                "value": str(mass),
            },
        )

    def _parse_inertial(xml_element):
        if xml_element is None:
            return None

        inertial = Inertial()
        inertial.origin = URDF._parse_origin(xml_element.find("origin"))
        inertial.inertia = URDF._parse_inertia(xml_element.find("inertia"))
        inertial.mass = URDF._parse_mass(xml_element.find("mass"))

        return inertial

    def _write_inertial(self, xml_parent, inertial):
        if inertial is None:
            return

        xml_element = etree.SubElement(xml_parent, "inertial")

        self._write_origin(xml_element, inertial.origin)
        self._write_mass(xml_element, inertial.mass)
        self._write_inertia(xml_element, inertial.inertia)

    def _parse_link(xml_element):
        link = Link(name=xml_element.attrib["name"])

        link.inertial = URDF._parse_inertial(xml_element.find("inertial"))

        for v in xml_element.findall("visual"):
            link.visuals.append(URDF._parse_visual(v))

        for c in xml_element.findall("collision"):
            link.collisions.append(URDF._parse_collision(c))

        return link

    def _write_link(self, xml_parent, link):
        xml_element = etree.SubElement(
            xml_parent,
            "link",
            attrib={
                "name": link.name,
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

        xyz = xml_element.get("xyz", "1 0 0")
        return np.array(list(map(float, xyz.split())))

    def _write_axis(self, xml_parent, axis):
        if axis is None:
            return

        etree.SubElement(xml_parent, "axis", attrib={"xyz": " ".join(map(str, axis))})

    def _parse_limit(xml_element):
        if xml_element is None:
            return None

        limit = Limit()
        limit.effort = _str2float(xml_element.get("effort", default=None))
        limit.velocity = _str2float(xml_element.get("velocity", default=None))
        limit.lower = _str2float(xml_element.get("lower", default=None))
        limit.upper = _str2float(xml_element.get("upper", default=None))

        return limit

    def _write_limit(self, xml_parent, limit):
        if limit is None:
            return

        attrib = {}
        if limit.effort is not None:
            attrib["effort"] = str(limit.effort)
        if limit.velocity is not None:
            attrib["velocity"] = str(limit.velocity)
        if limit.lower is not None:
            attrib["lower"] = str(limit.lower)
        if limit.upper is not None:
            attrib["upper"] = str(limit.upper)

        etree.SubElement(
            xml_parent,
            "limit",
            attrib=attrib,
        )

    def _parse_dynamics(xml_element):
        if xml_element is None:
            return None

        dynamics = Dynamics()
        dynamics.damping = xml_element.get("damping", default=None)
        dynamics.friction = xml_element.get("friction", default=None)

        return dynamics

    def _write_dynamics(self, xml_parent, dynamics):
        if dynamics is None:
            return

        attrib = {}
        if dynamics.damping is not None:
            attrib["damping"] = str(dynamics.damping)
        if dynamics.friction is not None:
            attrib["friction"] = str(dynamics.friction)

        etree.SubElement(
            xml_parent,
            "dynamics",
            attrib=attrib,
        )

    def _parse_joint(xml_element):
        joint = Joint(name=xml_element.attrib["name"])

        joint.type = xml_element.get("type", default=None)
        joint.parent = xml_element.find("parent").get("link")
        joint.child = xml_element.find("child").get("link")
        joint.origin = URDF._parse_origin(xml_element.find("origin"))
        joint.axis = URDF._parse_axis(xml_element.find("axis"))
        joint.limit = URDF._parse_limit(xml_element.find("limit"))
        joint.dynamics = URDF._parse_dynamics(xml_element.find("dynamics"))

        return joint

    def _write_joint(self, xml_parent, joint):
        xml_element = etree.SubElement(
            xml_parent,
            "joint",
            attrib={
                "name": joint.name,
                "type": joint.type,
            },
        )

        etree.SubElement(xml_element, "parent", attrib={"link": joint.parent})
        etree.SubElement(xml_element, "child", attrib={"link": joint.child})
        self._write_origin(xml_element, joint.origin)
        self._write_axis(xml_element, joint.axis)
        self._write_limit(xml_element, joint.limit)
        self._write_dynamics(xml_element, joint.dynamics)

    def _parse_robot(xml_element):
        robot = Robot(name=xml_element.attrib["name"])

        for l in xml_element.findall("link"):
            robot.links.append(URDF._parse_link(l))
        for j in xml_element.findall("joint"):
            robot.joints.append(URDF._parse_joint(j))

        return robot

    def _validate_robot(self, robot):
        if robot is not None:
            if robot.name is None:
                raise URDFIncompleteError(f"The <robot> tag misses a 'name' attribute.")
            elif len(robot.name) == 0:
                raise URDFIncompleteError(
                    f"The <robot> tag has an empty 'name' attribute."
                )

    def handleError(self, error):
        self.errors.append(error)
        if not type(error) in self.maskedErrors:
            raise

    def ignoreErrors(self, *args):
        """Add exceptions to the mask for ignoring or clear the mask if None given.
        You call c.ignoreErrors(e1, e2, ... ) if you want the loader to ignore those
        exceptions and continue loading whatever it can. If you want to empty the
        mask so all exceptions abort the load just call c.ignoreErrors(None).
        """
        if args == [None]:
            self.maskedErrors = []
        else:
            for e in args:
                self.maskedErrors.append(e)

    def _write_robot(self, robot):
        xml_element = etree.Element("robot", attrib={"name": robot.name})
        for link in robot.links:
            self._write_link(xml_element, link)
        for joint in robot.joints:
            self._write_joint(xml_element, joint)

        return xml_element

    def from_xml_file(fname, **kwargs):
        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(fname, parser)

        root = tree.getroot()

        if not "mesh_dir" in kwargs:
            kwargs["mesh_dir"] = os.path.dirname(fname)

        return URDF(xml_root=root, **kwargs)

    def contains(self, key, value, element=None):
        if element is None:
            element = self.robot

        result = False
        for field in element.__dataclass_fields__:
            field_value = getattr(element, field)
            if is_dataclass(field_value):
                result = result or self.contains(
                    key=key, value=value, element=field_value
                )
            elif (
                isinstance(field_value, list)
                and len(field_value) > 0
                and is_dataclass(field_value[0])
            ):
                for field_value_element in field_value:
                    result = result or self.contains(
                        key=key, value=value, element=field_value_element
                    )
            else:
                if key == field and value == field_value:
                    result = True
        return result

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

        if joint.type == "revolute":
            matrix = origin @ tra.rotation_matrix(q, joint.axis)
        elif joint.type == "prismatic":
            matrix = origin @ tra.translation_matrix(q * joint.axis)
        else:
            matrix = origin

        return matrix

    def update_trimesh_scene(self, trimesh_scene, configuration):
        # TODO: keep track of non-actuated joints
        if len(configuration) != len(self.robot.joints):
            raise ValueError(
                f"Dimensionality of configuration ({len(configuration)}) doesn't match number of actuated joints ({len(self.robot.joints)})."
            )

        for j, q in zip(self.robot.joints, configuration):
            matrix = self._forward_kinematics_joint(j, q=q)

            trimesh_scene.graph.update(
                frame_from=j.parent, frame_to=j.child, matrix=matrix
            )

    def _add_visual_to_scene(self, s, v, link_name, load_geometry=True):
        origin = v.origin if v.origin is not None else np.eye(4)

        if v.geometry is not None:
            new_s = None

            if v.geometry.box is not None:
                new_s = trimesh.Scene(
                    [trimesh.creation.box(extents=v.geometry.box.size)]
                )
            elif v.geometry.sphere is not None:
                new_s = trimesh.Scene(
                    [trimesh.creation.uv_sphere(radius=v.geometry.sphere.radius)]
                )
            elif v.geometry.cylinder is not None:
                new_s = trimesh.Scene(
                    [
                        trimesh.creation.cylinder(
                            radius=v.geometry.cylinder.radius,
                            height=v.geometry.cylinder.length,
                        )
                    ]
                )
            elif v.geometry.mesh is not None and load_geometry:
                new_filename = self._filename_handler(fname=v.geometry.mesh.filename)

                if os.path.isfile(new_filename):
                    print(f"Loading {v.geometry.mesh.filename} as {new_filename}")
                    # try:
                    # os.path.join(self.mesh_dir, v.geometry.mesh.filename)
                    new_s = trimesh.load(new_filename, force="scene")

                    # scale mesh appropriately
                    if v.geometry.mesh.scale is not None:
                        if isinstance(v.geometry.mesh.scale, float):
                            new_s = new_s.scaled(v.geometry.mesh.scale)
                        elif isinstance(v.geometry.mesh.scale, np.ndarray):
                            if not np.all(
                                v.geometry.mesh.scale == v.geometry.mesh.scale[0]
                            ):
                                print(
                                    f"Warning: Can't scale axis independently, will use the first entry of '{v.geometry.mesh.scale}'"
                                )
                            new_s = new_s.scaled(v.geometry.mesh.scale[0])
                        else:
                            print(
                                f"Warning: Can't interpret scale '{v.geometry.mesh.scale}'"
                            )
                    # except Exception as e:
                    #     print(e)
                    #     pass
                else:
                    print(f"Can't find {new_filename}")

            if new_s is not None:
                for name, geom in new_s.geometry.items():
                    s.add_geometry(
                        geometry=geom,
                        parent_node_name=link_name,
                        transform=origin @ new_s.graph.get(name)[0],
                    )

    def get_default_configuration(self):
        config = []
        for j in self.robot.joints:
            if j.type == "revolute" or j.type == "prismatic":
                if j.limit is not None:
                    config.append(j.limit.lower + 0.5 * (j.limit.upper - j.limit.lower))
                else:
                    config.append(0.0)
            elif j.type == "continuous" or j.type == "fixed":
                config.append(0.0)
            elif j.type == "floating":
                config.append([0.0] * 6)
            elif j.type == "planar":
                config.append([0.0] * 2)

        return np.array(config)

    def _create_scene(
        self, configuration=None, use_collision_geometry=False, load_geometry=True
    ):
        s = trimesh.scene.Scene(base_frame=self._determine_base_link())

        configuration = (
            self.get_default_configuration() if configuration is None else configuration
        )
        assert len(configuration) == len(self.robot.joints)

        for j, q in zip(self.robot.joints, configuration):
            matrix = self._forward_kinematics_joint(j, q=q)

            s.graph.update(frame_from=j.parent, frame_to=j.child, matrix=matrix)

        for l in self.robot.links:
            s.graph.nodes.add(l.name)

            meshes = l.collisions if use_collision_geometry else l.visuals
            for m in meshes:
                self._add_visual_to_scene(
                    s, m, link_name=l.name, load_geometry=load_geometry
                )

        return s

    def _successors(self, node):
        """
        Get all nodes of the scene that succeeds a specified node.

        Parameters
        ------------
        node : any
          Hashable key in `scene.graph`

        Returns
        -----------
        subnodes : set[str]
          Set of nodes.
        """
        # get every node that is a successor to specified node
        # this includes `node`
        return self._scene.graph.transforms.successors(node)

    def _create_subrobot(self, robot_name, root_link_name):
        subrobot = Robot(name=robot_name)
        subnodes = self._successors(node=root_link_name)

        if len(subnodes) > 0:
            for node in subnodes:
                if node in self.link_map:
                    subrobot.links.append(copy.deepcopy(self.link_map[node]))
            for joint_name, joint in self.joint_map.items():
                if joint.parent in subnodes and joint.child in subnodes:
                    subrobot.joints.append(copy.deepcopy(self.joint_map[joint_name]))

        return subrobot

    def split_along_joints(self, joint_type="floating"):
        result = [
            (
                np.eye(4),
                URDF(
                    robot=copy.deepcopy(self.robot),
                    load_meshes=False,
                    generate_scene_graph=False,
                ),
            )
        ]

        # find all freejoints
        joint_names = [j.name for j in self.robot.joints if j.type == joint_type]
        for joint_name in joint_names:
            root_link = self.link_map[self.joint_map[joint_name].child]
            new_robot = self._create_subrobot(
                robot_name=root_link.name,
                root_link_name=root_link.name,
            )

            result.append(
                (
                    self._scene.graph.get(root_link.name)[0],
                    URDF(
                        robot=new_robot, load_meshes=False, generate_scene_graph=False
                    ),
                )
            )

            # remove links and joints from
            for j in new_robot.joints:
                result[0][-1].robot.joints.remove(result[0][-1].joint_map[j.name])
            for l in new_robot.links:
                result[0][-1].robot.links.remove(result[0][-1].link_map[l.name])

        return result

    def validate_filenames(self):
        for l in self.robot.links:
            meshes = [
                m.geometry.mesh
                for m in l.collisions + l.visuals
                if m.geometry.mesh is not None
            ]
            for m in meshes:
                _logger.debug(m.filename, "-->", self._filename_handler(m.filename))
                if not os.path.isfile(self._filename_handler(m.filename)):
                    return False
        return True

    def write_xml(self):
        xml_element = self._write_robot(self.robot)
        return etree.ElementTree(xml_element)

    def write_xml_string(self, **kwargs):
        xml_element = self.write_xml()
        return etree.tostring(xml_element, xml_declaration=True, *kwargs)

    def write_xml_file(self, fname):
        xml_element = self.write_xml()
        xml_element.write(fname, xml_declaration=True, pretty_print=True)
