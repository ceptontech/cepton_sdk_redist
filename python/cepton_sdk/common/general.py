import copy
import ctypes
import functools

import numpy

import cepton_sdk.common.c
import cepton_util.common

_all_builder = cepton_util.common.AllBuilder(__name__)

from cepton_util.common import AllBuilder, SimpleTimer  # noqa isort:skip


def static_vars(**kwargs):
    """Add static variables to function.
    """
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


class single_cache:
    """Cache result for function returning single value."""

    def __init__(self, func):
        self._func = func
        self._result = None
        functools.update_wrapper(self, func)

    def __call__(self, *args, **kwargs):
        if self._result is None:
            self._result = self._func(*args, **kwargs)
        return copy.deepcopy(self._result)


def numpy_property(func, **kwargs):
    """Makes returned numpy object immutable to avoid modifying temporary value."""
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        result.flags.writeable = False
        return result
    return property(wrapper, **kwargs)


class ToDictMixin:
    """Used for simple JSON and string conversion."""
    @classmethod
    def _get_dict_member_names(cls):
        raise NotImplementedError()

    @classmethod
    def _from_dict_value(cls, member_name, dict_value):
        value = dict_value
        return value

    def _to_dict_value(self, member_name, value):
        dict_value = value
        return dict_value

    def update_from_dict(self, d, deep_copy=True, member_names=None):
        if member_names is None:
            member_names = list(d.keys())

        for member_name in member_names:
            dict_value = d[member_name]
            value = self._from_dict_value(member_name, dict_value)
            if deep_copy:
                value = copy.deepcopy(value)
            setattr(self, member_name, value)

    @classmethod
    def from_dict(cls, d, **kwargs):
        obj = cls()
        obj.update_from_dict(d, **kwargs)
        return obj

    def to_dict(self, deep_copy=True, member_names=None):
        if member_names is None:
            member_names = self._get_dict_member_names()

        d = {}
        for member_name in member_names:
            value = getattr(self, member_name)
            dict_value = self._to_dict_value(member_name, value)
            if deep_copy:
                dict_value = copy.deepcopy(dict_value)
            d[member_name] = dict_value
        return d


class C_Field:
    def __init__(self, field_name, field_type, field_width=None):
        self.name = field_name
        self.type = field_type
        self.width = field_width

    @classmethod
    def from_description(cls, descr):
        if len(descr) == 3:
            return cls(descr[0], descr[1], descr[2])
        else:
            return cls(descr[0], descr[1])


def _get_c_members(c_cls):
    return {x[0]: C_Field.from_description(x) for x in c_cls._fields_ if x[0]}


def _get_c_member_names(c_cls):
    return [x[0] for x in c_cls._fields_ if x[0]]


class ToCMixin:
    @classmethod
    def _get_c_class(cls):
        raise NotImplementedError()

    @classmethod
    def _from_c_value(cls, member_name, c_value):
        value = c_value
        return value

    def _to_c_value(self, member_name, value):
        c_member = self._get_c_member(member_name)
        if issubclass(c_member.type, ctypes.Array):
            if issubclass(c_member.type._type_, ctypes.c_char):
                if isinstance(value, str):
                    c_value = value.encode("utf-8")
                else:
                    c_value = value
            elif issubclass(c_member.type._type_, ctypes.c_wchar):
                c_value = value
            else:
                c_value = c_member.type(*value)
        else:
            c_value = value
        return c_value

    @classmethod
    def __get_c_members(cls):
        return _get_c_members(cls._get_c_class())

    @classmethod
    def _get_c_member_names(cls):
        return list(cls.__get_c_members().keys())

    @classmethod
    def _get_c_member(cls, member_name):
        return cls.__get_c_members()[member_name]

    @classmethod
    def from_c(cls, c_obj, deep_copy=True, member_names=None):
        if member_names is None:
            member_names = cls._get_c_member_names()

        obj = cls()
        for member_name in member_names:
            c_value = getattr(c_obj, member_name)
            value = cls._from_c_value(member_name, c_value)
            setattr(obj, member_name, value)
        return obj

    def to_c(self, c_type=None, deep_copy=True, member_names=None):
        if c_type is None:
            c_type = self._get_c_class()
        if member_names is None:
            member_names = self._get_c_member_names()

        c_cls = self._get_c_class()
        c_obj = c_cls()
        for member_name in member_names:
            try:
                value = getattr(self, member_name)
            except AttributeError:
                continue
            c_value = self._to_c_value(member_name, value)
            if deep_copy:
                c_value = copy.deepcopy(c_value)
            setattr(c_obj, member_name, c_value)

        return c_obj


class StructureOfArrays:
    """
    Group multiple arrays together and allow operations on all arrays
    simultaneously. Supports numpy operations as if it were a 1-d array.

    Avoid using instance member variables that are not listed as array members,
    since they will not be copied during operations. Instead, use class member
    variables.
    """

    def __init__(self, n=1):
        self._n = n

    @classmethod
    def _get_array_member_names(cls):
        raise NotImplementedError()

    def __len__(self):
        return self._n

    @property
    def size(self):
        return self._n

    @property
    def ndim(self):
        return 1

    @property
    def shape(self):
        return (self._n,)

    def __setattr__(self, name, value):
        cls = type(self)
        if hasattr(cls, name):
            # Static member
            return super().__setattr__(name, value)

        if hasattr(self, name):
            # Only set once
            raise AttributeError("Member already initialized!")
        if name in ["_n"]:
            # Special members
            return super().__setattr__(name, value)
        if name not in self._get_array_member_names():
            # Check member
            raise AttributeError(
                "Member `" + name + "` not listed in `_get_array_member_names`!")
        return super().__setattr__(name, value)

    @classmethod
    def get_common_names(cls, other):
        """Returns array member names common to both classes."""
        return list(set(cls._get_array_member_names()) &
                    set(other._get_array_member_names()))

    def update(self, other, names=None):
        """Copy fields from other."""
        if names is None:
            names = self.get_common_names(other)
        for name in names:
            getattr(self, name)[...] = getattr(other, name)

    @classmethod
    def convert(cls, other, **kwargs):
        """Create from other class."""
        self = cls(len(other))
        self.update(other, **kwargs)
        return self

    def _get_indices(self, key):
        indices = numpy.arange(len(self))[key]
        indices = numpy.reshape(indices, [-1])
        return indices

    def __getitem__(self, key):
        """Supports numpy style indexing as if object were 1-d array."""
        assert (not isinstance(key, tuple)), "Key must be 1-d."

        cls = type(self)
        result = cls(numpy.arange(len(self))[key].size)
        names = self._get_array_member_names()
        for name in names:
            getattr(result, name)[:] = getattr(self, name)[key]
        return result

    def __setitem__(self, key, other):
        """Supports numpy style assignment as if object were 1-d array."""
        assert (not isinstance(key, tuple)), "Key must be 1-d."

        names = self.get_common_names(other)
        for name in names:
            getattr(self, name)[key] = getattr(other, name)

    def assign(self, key, other, other_key):
        """
        Equivalent to
            self[key] = other[other_key]
        """
        assert (not isinstance(key, tuple)), "Key must be 1-d."
        assert (not isinstance(other_key, tuple)), "Key must be 1-d."

        names = self.get_common_names(other)
        for name in names:
            getattr(self, name)[key] = getattr(other, name)[other_key]

    @classmethod
    def combine(cls, other_list):
        """Combine list of objects into single object.

        Concatenates all member variables along first dimension.
        """
        other_list = list(other_list)
        if len(other_list) == 0:
            return cls(0)
        n = numpy.sum([len(x) for x in other_list])
        self = cls(n)
        offset = 0
        for i in range(len(other_list)):
            other = other_list[i]
            self[offset:offset + len(other)] = other
            offset += len(other)
        return self


class ToCArrayMixin:
    @classmethod
    def _get_c_class(cls):
        raise NotImplementedError()

    def _from_c_impl(self, data):
        raise NotImplementedError()

    def _to_c_impl(self, data):
        raise NotImplementedError()

    def update_from_c(self, c_a):
        if len(self) == 0:
            return
        data = cepton_sdk.common.c.convert_c_array_to_ndarray(len(self), c_a)
        self._from_c_impl(data)

    @classmethod
    def from_c(cls, n, c_a):
        self = cls(n)
        self.update_from_c(c_a)
        return self

    def to_c(self, c_type=None):
        if c_type is None:
            c_type = self._get_c_class()

        data = numpy.zeros(len(self), dtype=numpy.dtype(c_type))
        self._to_c_impl(data)
        c_ptr = cepton_sdk.c.convert_ndarray_to_c_array(data)
        return c_ptr


__all__ = _all_builder.get()
