import dynamixel_sdk as dxl

ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_CW_COMPLIANCE_MARGIN = 26
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27
ADDR_MX_CW_COMPLIANCE_SLOPE = 28
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29
ADDR_MX_GOAL_POSITION = 30
ADDR_MX_MOVING_SPEED = 32
ADDR_MX_PRESENT_POSITION = 36
ADDR_MX_PUNCH = 48
PROTOCOL_VERSION = 1.0
DXL_IDS = [1,2]
DEVICENAME = "COM11"
BAUDRATE = 1000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# robot nuovo: 150 240 150 150
# robot 4: 150 150 150 240

angle1 = int(150/300*1023)
packetHandler.write1ByteTxRx(
    portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_MOVING_SPEED, 30)
packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_GOAL_POSITION, angle1)


angle2 = int(110/300*1023)
packetHandler.write1ByteTxRx(
    portHandler, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_MOVING_SPEED, 30)
packetHandler.write2ByteTxRx(portHandler, 2, ADDR_MX_GOAL_POSITION, angle2)

angle3 = int(130/300*1023)
packetHandler.write1ByteTxRx(
        portHandler, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
    )
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
# packetHandler.write2ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
# packetHandler.write1ByteTxRx(portHandler, 1, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_MOVING_SPEED, 30)
packetHandler.write2ByteTxRx(portHandler, 3, ADDR_MX_GOAL_POSITION,angle3)

angle4 = int(130/300*1023)
packetHandler.write1ByteTxRx(
    portHandler, 4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE
)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0)
# packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32)
# packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32)
packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_MOVING_SPEED, 30)
packetHandler.write2ByteTxRx(portHandler, 4, ADDR_MX_GOAL_POSITION, angle4)