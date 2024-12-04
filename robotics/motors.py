from dataclasses import dataclass


@dataclass
class MotorBehavior:
    compliance_margin: int = 0
    compliance_slope: int = 32


@dataclass
class MotorsParameters:
    cw_behavior: MotorBehavior = MotorBehavior()
    ccw_behavior: MotorBehavior = MotorBehavior()

    moving_speed: int = 5
