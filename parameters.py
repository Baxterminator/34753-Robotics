import numpy as np

from robotics.motors import MotorsParameters
from robotics.types import Point3D, JointState, DEFAULT_PARAMS

ROBOT_MODEL = DEFAULT_PARAMS

# =============================================================================
# Exercise 3 - 4 circle parameters
# =============================================================================

R = 0.032  # Radius in meters
Xc = Point3D(0.15, 0.0, 0.12)  # Center of the circle in meters
Xaxis = Point3D(1, 0, 0)

# =============================================================================
# Exercise 11 parameters
# =============================================================================
MOTOR_PARAMS = MotorsParameters()
MOTOR_PARAMS.cw_behavior.compliance_margin = 3
MOTOR_PARAMS.cw_behavior.compliance_slope = 32
MOTOR_PARAMS.ccw_behavior.compliance_margin = 3
MOTOR_PARAMS.ccw_behavior.compliance_slope = 32
MOTOR_PARAMS.moving_speed = 10

MAX_VAL = 300  # In degrees
IDLE_JOINT = MAX_VAL // 2

ZEROS_POS = JointState(IDLE_JOINT, 240, IDLE_JOINT, IDLE_JOINT)  # In degrees
CAPTURE_POS = JointState(150, 110, 130, 130)  # In degrees

CAMERA_MATRIX = np.array([[1068.0, 0, 506.8],
                          [0, 1083.0, 371.8],
                          [0, 0, 1]])
DISTORSION_MATRIX = np.array([0.1158, 0.0412, 0, 0, 0])
