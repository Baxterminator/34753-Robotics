from typing import Tuple, List
from colorama import Fore, Style
import numpy as np

import sys 
import os

print(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from robot.kinematics import robot_ik, robot_fk, JointState, CartesianGoal, DEFAULT_PARAMS


def _test_eq(js1, js2) -> str:
    if js1 == js2:
        return Style.BRIGHT + Fore.GREEN + "V" + Fore.RESET + Style.RESET_ALL
    return Style.BRIGHT + Fore.RED + "X" + Fore.RESET + Style.RESET_ALL


def test_ik(js: JointState) -> Tuple[bool, List[JointState]]:
    print("".center(80, "-"))
    print(f"JointState= {js}")
    print(f"Original th24={js.q2 + js.q3 + js.q4}")
    print(f"Original cos(th3)={np.cos(js.q3)}")
    x = robot_fk(js)
    print(f"Forward kinematics:\n{x}")

    g = CartesianGoal(float(x[0, 0]), float(x[1, 0]), float(x[2, 0]), float(x[0, 3]), float(x[1, 3]), float(x[2, 3]))
    found_sols = robot_ik(g)
    print("Inverse kinematics:")
    has_good = False
    for i in range(len(found_sols)):
        if js == found_sols[i]:
            has_good = True
        print(f"{i}: {_test_eq(js, found_sols[i])} {found_sols[i]}")
    return has_good, found_sols


def ik_zeros():
    test_ik(JointState(0, 0, 0, 0))


def ik_horizontal():
    test_ik(JointState(0, np.deg2rad(-90), 0, 0))


def ik_random():
    def random_angle():
        return 2 * np.pi * np.random.rand() - np.pi

    js = JointState(
        random_angle(),
        random_angle(),
        random_angle(),
        random_angle()
    )
    solutions = test_ik(js)
    print("Delta between computed and found:")
    for i in range(len(solutions)):
        print(f"{i}: {js - solutions[i]}")


if __name__ == "__main__":
    # ik_horizontal()
    # ik_zeros()
    # ik_random()
    def random_angle():
        return 2 * np.pi * np.random.rand() - np.pi


    good_found = 0
    N = 1000
    for _ in range(N):
        [good, _] = test_ik(JointState(
            random_angle(),
            random_angle(),
            random_angle(),
            random_angle()
        ))
        if good:
            good_found += 1

    print("Results:")
    print("-------------")
    print(f"Succeed {good_found}/{N} -> {good_found / N:%}")
