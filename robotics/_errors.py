from enum import IntEnum
from dynamixel_sdk.robotis_def import *


class ResultCode(IntEnum):
    # General
    UNAVAILABLE = COMM_NOT_AVAILABLE
    SUCCESS = COMM_SUCCESS
    BUSY = COMM_PORT_BUSY

    # TX com errors
    TX_FAIL = COMM_TX_FAIL
    TX_ERROR = COMM_TX_ERROR

    # RX com errors
    RX_FAIL = COMM_RX_FAIL
    RX_CORRUPT = COMM_RX_CORRUPT
    RX_TIMEOUT = COMM_RX_TIMEOUT
    RX_WAITING = COMM_RX_WAITING

    def __str__(self):
        for name, val in ResultCode.__members__.items():
            if val == self:
                return f"{name} {self.value}"


class InvalidComError(RuntimeError):
    def __init__(self, code: ResultCode):
        super().__init__(f"Invalid communication with the robot: {code}")
