from dataclasses import dataclass
import numpy as np


@dataclass
class RobotParameters:
    d1: float
    a2: float
    a3: float
    a4: float


@dataclass
class CartesianGoal:
    rx: float = None
    ry: float = None
    rz: float = None
    x: float = None
    y: float = None
    z: float = None


def close_to(v1, v2) -> bool:
    return np.fabs(v1 - v2) < 1E-5


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

    def __eq__(self, other) -> bool:
        return close_to(self.q1, other.q1) and close_to(self.q2, other.q2) and close_to(self.q3, other.q3) and close_to(
            self.q4, other.q4)

    def __repr__(self):
        out = "JointState("
        out += f"q1= {self.q1: > 8.5f},"
        out += f"q2= {self.q2: > 8.5f},"
        out += f"q3= {self.q3: > 8.5f},"
        out += f"q4= {self.q4: > 8.5f})"
        return out

    def __str__(self):
        return self.__repr__()


DEFAULT_PARAMS = RobotParameters(0.05, 0.093, 0.093, 0.05)

T45 = np.array([
    [1, 0, 0, -0.015],
    [0, 1, 0, 0.035],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])