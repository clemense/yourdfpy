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

from .urdf import (
    Actuator,
    Box,
    Calibration,
    Collision,
    Color,
    Cylinder,
    Dynamics,
    Geometry,
    Inertial,
    Joint,
    Link,
    Limit,
    Material,
    Mesh,
    Mimic,
    Robot,
    SafetyController,
    Sphere,
    Texture,
    Transmission,
    TransmissionJoint,
    URDF,
    Visual,
    URDFError,
    URDFIncompleteError,
    URDFBrokenRefError,
    URDFSaveValidationError,
    URDFMalformedError,
    URDFUnsupportedError,
    filename_handler_null,
    filename_handler_ignore_directive,
    filename_handler_ignore_directive_package,
    filename_handler_add_prefix,
    filename_handler_absolute2relative,
    filename_handler_relative,
    filename_handler_relative_to_urdf_file,
    filename_handler_relative_to_urdf_file_recursive,
    filename_handler_meta,
    filename_handler_magic,
)
