from cepton_util import __author__, __version__  # noqa isort:skip

import cepton_sdk.common

_all_builder = cepton_sdk.common.AllBuilder(__name__)


def __check_version():
    c_version = cepton_sdk.c.get_version_string().split(".")[:2]
    version = __version__.split(".")[:2]
    if c_version != version:
        raise RuntimeError(
            "Library versions do not match: {} != {}".format(c_version, version))


try:
    import cepton_sdk.c
    from cepton_sdk.c import C_ErrorCode, C_Error, C_Warning
except OSError:
    # Allow loading python only parts of library
    pass
else:
    from cepton_sdk.api import *
    from cepton_sdk.core import *
    from cepton_sdk.point import *
    from cepton_sdk.sensor import *
    import cepton_sdk.capture_replay

    __check_version()

__all__ = _all_builder.get()
