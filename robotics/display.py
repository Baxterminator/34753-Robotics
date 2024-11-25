import time
from typing import List, Callable
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robotics.types import JointState, RobotParameters, DEFAULT_PARAMS
from robotics.kinematics import robot_fk


def displayJSPlot(js: List[JointState], title: str,
                  ax: plt.Axes | None = None,
                  robot: RobotParameters = DEFAULT_PARAMS,
                  anim: bool = False,
                  anim_callback: Callable[[plt.Axes], None] | None = None,
                  anim_hz: float = 10,
                  anim_loop: bool = True) -> plt.Axes:
    if ax is None:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        # plt.tight_layout()

    states = []
    for j in js:
        x = [0]
        y = [0]
        z = [0]
        for idx in range(1, 5):
            P = robot_fk(j, 0, idx, robot)
            x.append(P[0, 3])
            y.append(P[1, 3])
            z.append(P[2, 3])
        states.append([x, y, z])

    def plot_titles(_ax: Axes3D):
        _ax.set_xlabel("X-axis (m)")
        _ax.set_ylabel("Y-axis (m)")
        _ax.set_zlabel("Z-axis (m)")
        _ax.set_title(title)
        _ax.set_xlim(-0.2, 0.2)
        _ax.set_ylim(-0.2, 0.2)
        _ax.set_zlim(0, 0.3)
        _ax.legend()

    anim_duration = 1 / anim_hz if anim_hz != 0 else 100000
    if anim:
        first = True
        while (anim_loop or first) and (plt.get_fignums()):
            trace_X = []
            trace_Y = []
            trace_Z = []
            for s in states:
                ax.clear()

                # Making robot trace
                trace_X.append(s[0][-1])
                trace_Y.append(s[1][-1])
                trace_Z.append(s[2][-1])
                ax.plot(trace_X, trace_Y, trace_Z, "k--", label="Path of the robot")

                # Plotting robot state
                ax.plot(s[0], s[1], s[2], 'ro-', )

                # Formatting the plot
                if anim_callback is not None:
                    anim_callback(ax)
                plot_titles(ax)
                plt.show(block=False)
                plt.pause(anim_duration)
            first = False
            time.sleep(1)
    else:
        for s in states:
            ax.plot(s[0], s[1], s[2], 'ro-', )
        plot_titles(ax)
        ax.set_xlim(-0.2, 0.2)
        ax.set_ylim(-0.2, 0.2)
        ax.set_zlim(0, 0.3)
        plt.show(block=False)
        plt.pause(anim_duration)
    return ax


def display_table(lines: List[List[str]], width: int = 20) -> None:
    full_width = len(lines[0]) * (width + 1) + 1
    print("".center(full_width, "-"))
    for line in lines:
        print("|", end="")
        for col in line:
            print(col.center(20) + "|", end="")
        print()
    print("".center(full_width, "-"))
    print()
