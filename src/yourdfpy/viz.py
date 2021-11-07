"""
Script for visualizing a robot from a URDF.
"""

import argparse
import logging
import sys

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


def main(args):
    """Wrapper allowing string arguments in a CLI fashion.

    Args:
      args (List[str]): command line parameters as list of strings
          (for example  ``["--verbose", "42"]``).
    """
    args = parse_args(args)
    setup_logging(args.loglevel)
    urdf_model = URDF.load(args.input)
    urdf_model.show(collision_geometry=args.collision)


def run():
    """Calls :func:`main` passing the CLI arguments extracted from :obj:`sys.argv`.

    This function can be used as entry point to create console scripts with setuptools.
    """
    main(sys.argv[1:])


if __name__ == "__main__":
    run()
