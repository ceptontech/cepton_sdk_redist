import copy
import ctypes
import functools

import numpy


def static_vars(**kwargs):
    """Add static variables to function.
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


class single_cache(object):
    def __init__(self, func):
        self.func = func
        self.result = None
        functools.update_wrapper(self, func)

    def __call__(self, *args, **kwargs):
        if self.result is None:
            self.result = self.func(*args, **kwargs)
        return copy.deepcopy(self.result)