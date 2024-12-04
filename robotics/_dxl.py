import glob
import sys
from typing import Tuple, Any, List
import dynamixel_sdk as dxl
import serial
from serial.tools.list_ports import comports

from ._errors import *
from .motors import MotorsParameters


class DXLFlags(IntEnum):
    MX_TORQUE_ENABLE = 24
    MX_CW_COMPLIANCE_MARGIN = 26
    MX_CCW_COMPLIANCE_MARGIN = 27
    MX_CW_COMPLIANCE_SLOPE = 28
    MX_CCW_COMPLIANCE_SLOPE = 29
    MX_GOAL_POSITION = 30
    MX_MOVING_SPEED = 32
    MX_PRESENT_POSITION = 36
    MX_PUNCH = 48


DXLLength = {
    DXLFlags.MX_TORQUE_ENABLE: 1,
    DXLFlags.MX_CW_COMPLIANCE_MARGIN: 2,
    DXLFlags.MX_CCW_COMPLIANCE_MARGIN: 2,
    DXLFlags.MX_CW_COMPLIANCE_SLOPE: 1,
    DXLFlags.MX_CCW_COMPLIANCE_SLOPE: 1,
    DXLFlags.MX_GOAL_POSITION: 2,
    DXLFlags.MX_MOVING_SPEED: 2,
    DXLFlags.MX_PRESENT_POSITION: 2,
    DXLFlags.MX_PUNCH: 2,
}


class DXLWrapper:
    def __init__(self, interface: str, baud_rate: int, motors_param: MotorsParameters, protocol=1.0):
        # Parameters
        self._interface = interface
        self._protocol_version = protocol
        self._params = motors_param
        self._baud_rate = baud_rate
        self._real_robot = self._interface != "/dev/null"

        # Configuring components
        if self._real_robot:
            self._port: dxl.PortHandler = dxl.PortHandler(self._interface)  # type: ignore
            self._protocol: dxl.PacketHandler = dxl.PacketHandler(self._protocol_version)  # type: ignore

    def __enter__(self):
        print(f"Opening connection with the robot on `{self._interface}` with baud rate {self._baud_rate:_d}")
        if self._real_robot:
            self._port.openPort()
            self._port.setBaudRate(self._baud_rate)
            if not self.ping_robot():
                raise RuntimeError("Could not connect to robot!")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("Closing the connection with the robot")
        if self._real_robot:
            self._port.closePort()

    def send_command(self, motor_id: int, flag: DXLFlags, value: int) -> Tuple[ResultCode, Any]:
        # If no real robot
        if not self._real_robot:
            return ResultCode.SUCCESS, 0

        # Get flag and corresponding length
        n_bytes = DXLLength[flag]

        # Verify value in range
        max_value = 2 ** (8 * n_bytes - 1)
        min_value = -max_value
        max_value -= 1
        if value < min_value or max_value < value:
            raise RuntimeError("Given value is outside bounds of the given port")

        match n_bytes:
            case 1:
                [result, error] = self._protocol.write1ByteTxRx(self._port, motor_id, flag, value)
            case 2:
                [result, error] = self._protocol.write2ByteTxRx(self._port, motor_id, flag, value)
            case 4:
                [result, error] = self._protocol.write4ByteTxRx(self._port, motor_id, flag, value)
            case _:
                raise RuntimeError(f"Flag {flag} has no known equivalent bytes length")
        result_code = ResultCode(result)
        if result_code != ResultCode.SUCCESS:
            raise InvalidComError(result_code)
        return result_code, error

    def get_value(self, motor_id: int, flag: DXLFlags) -> Tuple[int, ResultCode, Any]:
        # If no real robot
        if not self._real_robot:
            return 0, ResultCode.SUCCESS, 0

        # Get flag and corresponding length
        n_bytes = DXLLength[flag]

        match n_bytes:
            case 1:
                [val, result, error] = self._protocol.read1ByteTxRx(self._port, motor_id, flag)
            case 2:
                [val, result, error] = self._protocol.read2ByteTxRx(self._port, motor_id, flag)
            case 4:
                [val, result, error] = self._protocol.read4ByteTxRx(self._port, motor_id, flag)
            case _:
                raise RuntimeError(f"Flag {flag} has no known equivalent bytes length")
        result_code = ResultCode(result)
        if result_code != ResultCode.SUCCESS:
            raise InvalidComError(result_code)
        return val, result_code, error

    @staticmethod
    def deg_to_int_angle(val: float):
        return int(val % 300 / 300 * 1023)

    @staticmethod
    def int_to_deg_angle(val: int):
        return val * 300 / 1023

    # =========================================================================
    # Utils functions
    # =========================================================================

    def ping_robot(self) -> bool:
        if not self._real_robot:
            return True
        [model_number, result, error] = self._protocol.ping(self._port, 1)
        result_code = ResultCode(result)
        return result_code == ResultCode.SUCCESS

    def configure_robot(self):
        print("Configuring robot")
        for i in range(1, 5):
            print(f"\t - Configuring motor {i}")
            # CW movement
            self.send_command(i, DXLFlags.MX_CW_COMPLIANCE_MARGIN, self._params.cw_behavior.compliance_margin)
            self.send_command(i, DXLFlags.MX_CW_COMPLIANCE_SLOPE, self._params.cw_behavior.compliance_slope)

            # CCW movement
            self.send_command(i, DXLFlags.MX_CCW_COMPLIANCE_MARGIN, self._params.ccw_behavior.compliance_margin)
            self.send_command(i, DXLFlags.MX_CCW_COMPLIANCE_SLOPE, self._params.ccw_behavior.compliance_slope)

            # Speed
            self.send_command(i, DXLFlags.MX_MOVING_SPEED, self._params.moving_speed)

    def get_motors_pos(self) -> List[int]:
        out = [
            self.get_value(i, DXLFlags.MX_PRESENT_POSITION)[0]
            for i in range(1, 5)
        ]
        return out
