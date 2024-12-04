from dataclasses import dataclass
from typing import TypeVar, Generic

import numpy as np

from robotics.math import close_to


@dataclass
class RobotParameters:
    d1: float
    a2: float
    a3: float
    a4: float

    dx_c: float
    dy_c: float


@dataclass
class Point3D:
    x: float
    y: float
    z: float

    def to_arr(self) -> np.ndarray:
        return np.array([
            self.x, self.y, self.z
        ]).T


@dataclass
class CartesianGoal(Point3D):
    rx: float = None
    ry: float = None
    rz: float = None


@dataclass
class JointState:
    q1: float
    q2: float
    q3: float
    q4: float

    def __sub__(self, other):
        return JointState(
            self.q1 - other.q1,
            self.q2 - other.q2,
            self.q3 - other.q3,
            self.q4 - other.q4,
        )

    def close_to(self, other: "JointState", tol: float) -> bool:
        return close_to(self.q1, other.q1, tol) and close_to(self.q2, other.q2, tol) and close_to(self.q3, other.q3,
                                                                                                  tol) and close_to(
            self.q4, other.q4, tol)

    def __eq__(self, other) -> bool:
        return self.close_to(other)

    def __repr__(self):
        out = "JointState("
        out += f"q1= {self.q1: > 8.5f},"
        out += f"q2= {self.q2: > 8.5f},"
        out += f"q3= {self.q3: > 8.5f},"
        out += f"q4= {self.q4: > 8.5f})"
        return out

    def __str__(self):
        return self.__repr__()

    def __getitem__(self, key):
        match key:
            case 1:
                return self.q1
            case 2:
                return self.q2
            case 3:
                return self.q3
            case 4:
                return self.q4
            case _:
                raise RuntimeError(f"Accessing invalid joint value {key} ")

    @staticmethod
    def from_vector(v: np.ndarray) -> "JointState":
        assert np.shape(v)[0] >= 4
        return JointState(float(v[0]), float(v[1]), float(v[2]), float(v[3]))


T = TypeVar("T")


@dataclass
class InterpolationNode(Generic[T]):
    t: float
    q: T
    dq: T
    ddq: T


DEFAULT_PARAMS = RobotParameters(0.05, 0.093, 0.093, 0.05, -0.015, 0.045)
