"""
Script for visualizing a robot from a URDF.
"""

import sys
import time
import logging
import argparse
import numpy as np
from functools import partial

from yourdfpy import __version__
from yourdfpy import URDF

__author__ = "Clemens Eppner"
__copyright__ = "Clemens Eppner"
__license__ = "MIT"

_logger = logging.getLogger(__name__)


def parse_args(args):
    """Parse command line parameters

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--help"]``).

    Returns:
      :obj:`argparse.Namespace`: command line parameters namespace
    """
    parser = argparse.ArgumentParser(description="Visualize a URDF model.")
    parser.add_argument(
        "--version",
        action="version",
        version="yourdfpy {ver}".format(ver=__version__),
    )
    parser.add_argument(
        "input",
        help="URDF file name.",
    )
    parser.add_argument(
        "-c",
        "--configuration",
        nargs="+",
        type=float,
        help="Configuration of the visualized URDF model.",
    )
    parser.add_argument(
        "--collision",
        action="store_true",
        help="Use collision geometry for the visualized URDF model.",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="Animate model by interpolating through all actuated joint limits.",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        dest="loglevel",
        help="set loglevel to INFO",
        action="store_const",
        const=logging.INFO,
    )
    parser.add_argument(
        "-vv",
        "--very-verbose",
        dest="loglevel",
        help="set loglevel to DEBUG",
        action="store_const",
        const=logging.DEBUG,
    )
    return parser.parse_args(args)


def setup_logging(loglevel):
    """Setup basic logging.

    Args:
      loglevel (int): minimum loglevel for emitting messages
    """
    logformat = "[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
    logging.basicConfig(
        level=loglevel, stream=sys.stdout, format=logformat, datefmt="%Y-%m-%d %H:%M:%S"
    )


def generate_joint_limit_trajectory(urdf_model, loop_time):
    """Generate a trajectory for all actuated joints that interpolates between joint limits.
    For continuous joint interpolate between [0, 2 * pi].

    Args:
        urdf_model (yourdfpy.URDF): _description_
        loop_time (float): Time in seconds to loop through the trajectory.

    Returns:
        dict: A dictionary over all actuated joints with list of configuration values.
    """
    trajectory_via_points = {}
    for joint_name in urdf_model.actuated_joint_names:
        if urdf_model.joint_map[joint_name].type.lower() == "continuous":
            via_point_0 = 0.0
            via_point_2 = 2.0 * np.pi
            via_point_1 = (via_point_2 - via_point_0) / 2.0
        else:
            limit_lower = (
                urdf_model.joint_map[joint_name].limit.lower
                if urdf_model.joint_map[joint_name].limit.lower is not None
                else -np.pi
            )
            limit_upper = (
                urdf_model.joint_map[joint_name].limit.upper
                if urdf_model.joint_map[joint_name].limit.upper is not None
                else +np.pi
            )
            via_point_0 = limit_lower
            via_point_1 = limit_upper
            via_point_2 = limit_lower

        trajectory_via_points[joint_name] = np.array(
            [
                via_point_0,
                via_point_1,
                via_point_2,
            ]
        )
    times = np.linspace(0.0, 1.0, int(loop_time * 100.0))
    bins = np.arange(3) / 2.0

    # Compute alphas for each time
    inds = np.digitize(times, bins, right=True)
    inds[inds == 0] = 1
    alphas = (bins[inds] - times) / (bins[inds] - bins[inds - 1])

    # Create the new interpolated trajectory
    trajectory = {}
    for k in trajectory_via_points:
        trajectory[k] = (
            alphas * trajectory_via_points[k][inds - 1]
            + (1.0 - alphas) * trajectory_via_points[k][inds]
        )

    return trajectory


def viewer_callback(scene, urdf_model, trajectory, loop_time):
    frame = int(100.0 * (time.time() % loop_time))
    cfg = {k: trajectory[k][frame] for k in trajectory}

    urdf_model.update_cfg(configuration=cfg)


def main(args):
    """Wrapper allowing string arguments in a CLI fashion.

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--verbose", "42"]``).
    """
    args = parse_args(args)
    setup_logging(args.loglevel)

    if args.collision:
        urdf_model = URDF.load(
            args.input, build_collision_scene_graph=True, load_collision_meshes=True
        )
    else:
        urdf_model = URDF.load(args.input)

    if args.configuration:
        urdf_model.update_cfg(args.configuration)

    callback = None
    if args.animate:
        loop_time = 6.0
        callback = partial(
            viewer_callback,
            urdf_model=urdf_model,
            loop_time=loop_time,
            trajectory=generate_joint_limit_trajectory(
                urdf_model=urdf_model, loop_time=loop_time
            ),
        )

    urdf_model.show(
        collision_geometry=args.collision,
        callback=callback,
    )


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`.

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
