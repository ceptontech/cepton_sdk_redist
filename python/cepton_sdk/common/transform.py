import numpy
import transforms3d
import transforms3d.quaternions

from cepton_sdk.common import *

_all_builder = AllBuilder(__name__)


class Quaternion:
    def __init__(self):
        self.s = 0.0
        self.v = numpy.zeros([3])

    @classmethod
    def from_vector(cls, v, scalar_first=False):
        self = cls()
        if scalar_first:
            self.s = v[0]
            self.v[:] = v[1:]
        else:
            self.v[:] = v[:3]
            self.s = v[3]
        return self

    def to_vector(self, scalar_first=False):
        result = numpy.zeros([4])
        if scalar_first:
            result[0] = self.s
            result[1:] = self.v
        else:
            result[:3] = self.v
            result[3] = self.s
        return result

    @classmethod
    def from_matrix(self, mat):
        return self.from_vector(
            transforms3d.quaternions.mat2quat(mat), scalar_first=True)

    def to_matrix(self):
        return transforms3d.quaternions.quat2mat(
            self.to_vector(scalar_first=True))

    def apply(self, v):
        return numpy.matmul(self.to_matrix(), v.transpose()).transpose()


class Transform3d:
    def __init__(self):
        self.rotation = Quaternion()
        self.translation = numpy.zeros([3])

    @classmethod
    def from_matrix(cls, mat):
        self = cls()
        self.rotation = Quaternion.from_matrix(mat[:3, :3])
        self.translation = mat[3, :3]
        return self

    def to_matrix(self):
        result = numpy.identity(4)
        result[:3, :3] = self.rotation.to_matrix()
        result[:3, 3] = self.translation
        return result

    def apply(self, v):
        return self.rotation.apply(v) + self.translation


__all__ = _all_builder.get()
