import sys
import glob
from typing import Dict, List

import cv2
import numpy as np
import serial

from serial.tools.list_ports_common import ListPortInfo


def get_all_ports() -> Dict[str, str]:
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z0-9]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = {}
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            info = ListPortInfo(port)
            result[port] = info.vid
        except (OSError, serial.SerialException):
            pass
    return result


def get_all_cam_index() -> List[int]:
    index = 10
    arr = []
    while index >= 0:
        try:
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                arr.append(np.max([index, 0]))
            cap.release()
        except:
            pass
        index -= 1
    arr.reverse()
    return arr
