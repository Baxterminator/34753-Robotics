from robotics.trajectories import circle_path
import matplotlib.pyplot as plt
from robotics.kinematics import robot_ik
from robotics.display import displayJSPlot, display_table

from parameters import R, Xc, Xaxis

if __name__ == "__main__":
    [phis, goals] = circle_path(R, Xc, Xaxis)
    joints_states = [robot_ik(goal)[0] for goal in goals]

    # Print goals to the table
    values = [["Point", "Goal X (m)", "Goal Y (m)", "Goal Z (m)", "Joint Q1 (rad)", "Joint Q2 (rad)", "Joint Q3 (rad)",
               "Joint Q4 (rad)"]]
    for i in range(len(goals)):
        line = [f"Point {i + 1}"]
        line += [f"{goals[i].x:.3f}", f"{goals[i].y:.3f}", f"{goals[i].z:.3f}"]
        line += [f"{joints_states[i].q1:.3f}", f"{joints_states[i].q2:.3f}", f"{joints_states[i].q3:.3f}",
                 f"{joints_states[i].q4:.3f}"]
        values.append(line)
    display_table(values)


    def display_circle(_ax: plt.Axes):
        _ax.plot([p.x for p in goals],
                 [p.y for p in goals],
                 [p.z for p in goals],
                 'b--',
                 label="Desired Circle"
                 )


    # Display
    ax = displayJSPlot(joints_states, "3D Robot Arm Tracking a Circle with Integrated Inverse Kinematics", anim=True,
                       anim_callback=display_circle)
    plt.show()
