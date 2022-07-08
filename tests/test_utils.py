import pytest
from functools import partial

from yourdfpy import urdf

__author__ = "Clemens Eppner"
__copyright__ = "Clemens Eppner"
__license__ = "MIT"


def test_filename_handler_absolute2relative():
    result = urdf.filename_handler_absolute2relative(
        fname="/a/b/c/d.urdf",
        dir="/a/b/",
    )
    assert result == "c/d.urdf"

    result = urdf.filename_handler_absolute2relative(
        fname="/a/b/c/d.urdf",
        dir="/c/d/",
    )
    assert result == "/a/b/c/d.urdf"


def test_filename_handler_add_prefix():
    result = urdf.filename_handler_add_prefix(
        fname="a/b/c/hoho.urdf",
        prefix="package://",
    )
    assert result == "package://a/b/c/hoho.urdf"


def test_filename_handler_ignore_directive():
    result = urdf.filename_handler_ignore_directive(fname="/a/b/c/d.urdf")
    assert result == "/a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive(fname="package://a/b/c/d.urdf")
    assert result == "a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive(fname="file://a/b/c/d.urdf")
    assert result == "a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive(fname="file:///a/b/c/d.urdf")
    assert result == "/a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive(fname="file:\\\\a\\b\\c\\d.urdf")
    assert result == "a\\b\\c\\d.urdf"


def test_filename_handler_ignore_directive_package():
    result = urdf.filename_handler_ignore_directive_package(fname="/a/b/c/d.urdf")
    assert result == "/a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive_package(
        fname="package://a/b/c/d.urdf"
    )
    assert result == "b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive_package(fname="file://a/b/c/d.urdf")
    assert result == "a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive_package(
        fname="file:///a/b/c/d.urdf"
    )
    assert result == "/a/b/c/d.urdf"

    result = urdf.filename_handler_ignore_directive_package(
        fname="file:\\\\a\\b\\c\\d.urdf"
    )
    assert result == "a\\b\\c\\d.urdf"


def test_filename_handler_magic():
    result = urdf.filename_handler_magic(fname="/a/b/c/d/e.urdf", dir="/a/")
    assert result == "/a/b/c/d/e.urdf"


def test_filename_handler_meta():
    result = urdf.filename_handler_meta(
        fname="/a/b/c/d/e.urdf",
        filename_handlers=[
            urdf.filename_handler_ignore_directive,
            partial(urdf.filename_handler_absolute2relative, dir="/a/"),
        ],
    )
    assert result == "/a/b/c/d/e.urdf"


def test_filename_handler_null():
    result = urdf.filename_handler_null(fname="a/b/c/d/e.urdf")
    assert result == "a/b/c/d/e.urdf"


def test_filename_handler_relative():
    result = urdf.filename_handler_relative(fname="d/e.urdf", dir="/a/b/c")
    assert result == "/a/b/c/d/e.urdf"


def test_filename_handler_relative_to_urdf_file():
    result = urdf.filename_handler_relative_to_urdf_file(
        fname="b/c/d.urdf", urdf_fname="/a/b.urdf"
    )
    assert result == "/a/b/c/d.urdf"


def test_filename_handler_relative_to_urdf_file_recursive():
    result = urdf.filename_handler_relative_to_urdf_file_recursive(
        fname="b/c/d.urdf", urdf_fname="/a/b.urdf", level=1
    )
    assert result == "/b/c/d.urdf"
