import cepton_util.common
import pathlib

version_path = pathlib.Path(__file__).resolve().parent / "VERSION"
__version__ = version_path.open().read().strip()

__author__ = "Cepton Technologies"

_all_builder = cepton_util.common.AllBuilder(__name__)

from cepton_util.common import *  # noqa isort:skip

__all__ = _all_builder.get()
