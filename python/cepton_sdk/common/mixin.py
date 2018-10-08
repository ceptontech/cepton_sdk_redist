import copy
import ctypes

import numpy


def numpy_property(func, **kwargs):
    """Makes returned numpy object immutable to avoid modifying temporary value."""
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        result.flags.writeable = False
        return result
    return property(wrapper, **kwargs)


class ToDictMixin:
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
        # TODO
        # c_member = self._get_c_member(member_name)
        # if issubclass(c_member.type, ctypes.Array):
        #     if issubclass(c_member.type._type_, c_char):
        #         value = c_value.decode("utf-8")
        #     else:
        #         value = list(c_value)
        value = c_value
        return value

    def _to_c_value(self, member_name, value):
        c_member = self._get_c_member(member_name)
        if issubclass(c_member.type, ctypes.Array):
            if issubclass(c_member.type._type_, c_char):
                if isinstance(value, str):
                    c_value = value.encode("utf-8")
                else:
                    c_value = value
            elif issubclass(c_member.type._type_, c_wchar):
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
    def _get_c_member(cls):
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

    def _setattr(self, name, value):
        super().__setattr__(name, value)

    def __setattr__(self, name, value):
        if hasattr(self, name):
            raise AttributeError("Member already initialized!")
        if name in ["_n"]:
            return self._setattr(name, value)
        if name not in self._get_array_member_names():
            raise AttributeError(
                "Member not listed in `_get_array_member_names`!")
        return self._setattr(name, value)

    @classmethod
    def from_parent(cls, other):
        """Cast from parent class to child class.

        Copies over all parent member variables.
        """
        parent_cls = type(other)

        obj = cls(len(other))
        for name in parent_cls._get_array_member_names():
            value = getattr(other, name)
            setattr(obj, name, value)
        return obj

    def _get_indices(self, key):
        indices = numpy.arange(len(self))[key]
        indices = numpy.reshape(indices, [-1])
        return indices

    def __getitem__(self, key):
        """Supports numpy style indexing as if object were 1-d array.
        """
        indices = self._get_indices(key)

        cls = type(self)
        selected_obj = cls(indices.size)
        for name in self._get_array_member_names():
            data = getattr(self, name)
            selected_data = data[indices, ...]
            selected_obj._setattr(name, selected_data)
        return selected_obj

    def __setitem__(self, key, other):
        """Supports numpy style assignment as if object were 1-d array.
        """
        if not isinstance(other, type(self)):
            raise TypeError("Incompatible types")

        indices = self._get_indices(key)

        for name in self._get_array_member_names():
            getattr(self, name)[indices, ...] = getattr(other, name)

    @classmethod
    def combine(cls, obj_list):
        """Combine list of objects into single object.

        Concatenates all member variables along first dimension.
        """
        for obj in obj_list:
            if not isinstance(obj, cls):
                raise TypeError("Incompatible types")

        obj_list = list(obj_list)
        if len(obj_list) == 0:
            return cls(0)

        n_combined = numpy.sum([len(x) for x in obj_list])
        combined_obj = cls(n_combined)
        for name in obj_list[0]._get_array_member_names():
            value_list = [getattr(x, name) for x in obj_list]
            try:
                combined_value = value_list[0].combine(value_list)
            except AttributeError:
                combined_value = numpy.concatenate(value_list, axis=0)
            combined_obj._setattr(name, combined_value)
        return combined_obj
