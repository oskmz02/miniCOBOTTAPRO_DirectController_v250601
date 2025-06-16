#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time


class Dynamixel:
    ver = "22.7.26"  # もしかしたら，ver違いで動かない可能性あり
    # 1byte groupSyncWriteが動かない
    #
    ## Control table address
    # Addr, Size, Name, RW, Default, Range, Unit
    # 8	1	Baud Rate	RW	1	0 ~ 7	-
    ADDR_RETURN_DELAY_TIME = 9  # 1 RW	250	0 ~ 254	2 [μsec]
    ADDR_DRIVE_MODE = 10  # 1 RW	0	0 ~ 5	-
    #     1: Reverse mode
    ADDR_OPERATING_MODE = 11  # 1 RW	3	0 ~ 16	-
    #     0: Current Control Mode
    #     1: Velocity Control Mode
    #     3: Position Control Mode
    #     16: PWM Control Mode (Voltage Control Mode)
    # 36	2	PWM Limit	RW	885	0 ~ 885	0.113 [%]
    # 38	2	Current Limit	RW	1,193	0 ~ 1,193	2.69 [mA]
    # 44	4	Velocity Limit	RW	200	0 ~ 1,023	0.229 [rev/min]
    # 48	4	Max Position Limit	RW	4,095	0 ~ 4,095	1 [pulse]
    # 52	4	Min Position Limit	RW	0	0 ~ 4,095	1 [pulse]
    ADDR_TORQUE_ENABLE = 64  # 1 RW	0	0 ~ 1	-
    ADDR_LED = 65  # 1 RW	0	0 ~ 1	-
    ADDR_STATUS_RETURN_LEVEL = 68  # 1 RW	2	0 ~ 2	-
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    #     0: PING Instruction
    #     1: PING Instruction, READ Instruction
    #     2: All Instructions
    ADDR_GOAL_PWM = 100  # 0 RW	-	-PWM Limit(36) ~ PWM Limit(36)	-
    ADDR_GOAL_CURRENT = 102  # 2 RW	-	-Current Limit(38) ~ Current Limit(38)	2.69 [mA]
    ADDR_GOAL_VELOCITY = (
        104  # 4 RW	-	-Velocity Limit(44) ~ Velocity Limit(44)	0.229 [rev/min]
    )
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = (
        116  # 4 RW	-	Min Position Limit(52) ~ Max Position Limit(48)	1 [pulse]
    )
    ADDR_PRESENT_PWM = 124  # 2 R	-	-	-
    ADDR_PRESENT_CURRENT = 126  # 2 R	-	-	2.69 [mA]
    ADDR_PRESENT_VELOCITY = 128  # 4	R	-	-	0.229 [rev/min]
    ADDR_PRESENT_POSITION = 132  # 4	R	-	-	1 [pulse]

    # Protocol version
    PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel
    # COMM_SUCCESS                = 0;            % Communication Success result value
    # COMM_TX_FAIL                = -1001;        % Communication Tx Failed

    def __init__(self, device_name, baud):
        self.DEVICENAME = device_name
        self.BAUDRATE = baud
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(Dynamixel.PROTOCOL_VERSION)
        self.getdata_result = True

        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            self.portHandler.closePort()
            quit()

        # Set port baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("Failed to change the baudrate")
            self.portHandler.closePort()
            quit()
        # print(len(self.DXL_IDs))

    def __del__(self):
        # Close port
        self.portHandler.closePort()

    def setRecommendedValue(self, DXL_IDs):
        self.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        self.writeReturnDelayTime(DXL_IDs, [5] * len(DXL_IDs))
        # self.writeStatusReturnLevel(DXL_IDs, [0] * len(DXL_IDs))
        self.writeStatusReturnLevel(
            DXL_IDs, [1] * len(DXL_IDs)
        )  # [2]に変えると取得データがバグる
        # self.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

    def readReturnDelayTime(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_RETURN_DELAY_TIME, 1, DXL_IDs)

    def readDriveMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_DRIVE_MODE, 1, DXL_IDs)

    def readOperatingMode(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_OPERATING_MODE, 1, DXL_IDs)

    def readTorqueEnable(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_TORQUE_ENABLE, 1, DXL_IDs)

    def readLED(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_LED, 1, DXL_IDs)

    def readStatusReturnLevel(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_STATUS_RETURN_LEVEL, 1, DXL_IDs)

    def writeReturnDelayTime(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_RETURN_DELAY_TIME,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    def writeDriveMode(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_DRIVE_MODE,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    def writeOperatingMode(self, DXL_IDs, data):
        """
        基本は下記URLと同じだろう
        https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#operating-mode11

        詳細はe-Manualからoperating modeを調べる
        https://emanual.robotis.com/docs/en/dxl/
        """
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_OPERATING_MODE,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    def writeTorqueEnable(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_TORQUE_ENABLE,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    """
    def writePositionDGain(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_POSITION_D_GAIN,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_comm_result == COMM_SUCCESS:
                print(f"ID:{DXL_IDs} Position D Gain:{data}")

    def writePositionPGain(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_POSITION_P_GAIN,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_comm_result == COMM_SUCCESS:
                print(f"ID:{DXL_IDs} Position P Gain:{data}")
    """

    def writeLED(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_LED,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    def writeStatusReturnLevel(self, DXL_IDs, data):
        for DXL_ID in DXL_IDs:
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler,
                DXL_ID,
                Dynamixel.ADDR_STATUS_RETURN_LEVEL,
                int(data[DXL_IDs.index(DXL_ID)]),
            )
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            time.sleep(0.05)

    """
     現在の状態を取得する関数
    """

    def readPresentPWM(self, DXL_IDs, data):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_PWM, 2, DXL_IDs)

    def readPresentVelocity(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_VELOCITY, 4, DXL_IDs)

    def readPresentPosition(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_POSITION, 4, DXL_IDs)

    def readPresentCurrent(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PRESENT_CURRENT, 2, DXL_IDs)

    def readPresentPositionDGain(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_POSITION_D_GAIN, 2, DXL_IDs)

    def readPresentPositionIGain(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_POSITION_I_GAIN, 2, DXL_IDs)

    def readProfileVelocity(self, DXL_IDs):
        return self.groupSyncRead(Dynamixel.ADDR_PROFILE_VELOCITY, 4, DXL_IDs)

    """
     ACTRの目標値を送信する関数
    """

    def writeGoalPWM(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_PWM, 2, DXL_IDs, data)

    def writeGoalCurrent(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_CURRENT, 2, DXL_IDs, data)

    def writeGoalVelocity(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_VELOCITY, 4, DXL_IDs, data)

    def writeGoalPosition(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_GOAL_POSITION, 4, DXL_IDs, data)

    """
    Dynamixel内PIDゲインの調整関数
    """

    def writePositionDGain(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_POSITION_D_GAIN, 2, DXL_IDs, data)

    def writePositionIGain(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_POSITION_I_GAIN, 2, DXL_IDs, data)

    def writePositionPGain(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_POSITION_P_GAIN, 2, DXL_IDs, data)

    """
    Dynamixel内プロファイル調整関数
    """

    def writeProfileAcceleration(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_PROFILE_ACCELERATION, 4, DXL_IDs, data)

    def writeProfileVelocity(self, DXL_IDs, data):
        self.groupSyncWrite(Dynamixel.ADDR_PROFILE_VELOCITY, 4, DXL_IDs, data)

    # def writeGoalCurPos(self, DXL_IDs, data):
    # self.groupSyncWrite(Dynamixel.ADDR_GOAL)

    def groupSyncWrite(self, ADDR, byte, DXL_IDs, data):
        groupSyncWritePacket = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR, byte
        )
        # self.groupSyncWriteGoalPosition.clearParam()
        for DXL_ID in DXL_IDs:
            iData = int(data[DXL_IDs.index(DXL_ID)])
            if byte == 4:
                param = [
                    iData & 0xFF,
                    (iData >> 8) & 0xFF,
                    (iData >> 16) & 0xFF,
                    (iData >> 24) & 0xFF,
                ]
            elif byte == 2:
                param = [iData & 0xFF, (iData >> 8) & 0xFF]
            else:
                param = iData & 0xFF
            # dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, self.signed_hex2int(param, 8))
            dxl_addparam_result = groupSyncWritePacket.addParam(DXL_ID, param)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID)
        dxl_comm_result = groupSyncWritePacket.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

    def groupSyncRead(self, ADDR, byte, DXL_IDs):
        data = [0] * len(DXL_IDs)
        groupSyncReadPacket = GroupSyncRead(
            self.portHandler, self.packetHandler, ADDR, byte
        )
        self.getdata_result_array = [False] * len(DXL_IDs)  # 最初に全部Falseで初期化
        self.getdata_result = True
        # groupSyncReadPacket.clearParam()
        for DXL_ID in DXL_IDs:
            dxl_addparam_result = groupSyncReadPacket.addParam(DXL_ID)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID)
        dxl_comm_result = groupSyncReadPacket.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        for DXL_ID in DXL_IDs:
            dxl_getdata_result = groupSyncReadPacket.isAvailable(DXL_ID, ADDR, byte)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL_ID)
                self.getdata_result = False
            num = DXL_IDs.index(DXL_ID)
            self.getdata_result_array[num] = dxl_getdata_result  # 各IDの通信可否を記録
            data[num] = groupSyncReadPacket.getData(DXL_ID, ADDR, byte)
            if byte == 4:
                data[num] = self.signed_hex2int(data[num], 32)
                # data[num] = float(data[num]+(data[num]>>31)*(1-0xFFFFFFFF))
            elif byte == 2:
                data[num] = self.signed_hex2int(data[num], 16)
            else:
                data[num] = data[num] & 0xFF
                # data[num] = self.signed_hex2int(data[num], 8)
        return data

    @staticmethod
    def signed_hex2int(signed_hex, digit):
        signed = 0x01 << (digit - 1)
        mask = 0x00
        for num in range(digit):
            mask = mask | (0x01 << num)
        signed_int = (
            (int(signed_hex ^ mask) * -1) - 1
            if (signed_hex & signed)
            else int(signed_hex)
        )
        return signed_int


class Test_Dynamixel:
    def __init__(self):
        pass

    """
    以下テストコード
    下記関数をtest.pyにコピーして使用することで，testが可能
    引数：
        dyn : Dynamixel class
        DXL_IDs : ACTR ID
    """

    def timeMesurement(self, func, num, arg1=None, arg2=None, arg3=None):
        start = time.time()
        if num == 0:
            ret = func()
        elif num == 1:
            ret = func(arg1)
        elif num == 2:
            ret = func(arg1, arg2)
        else:
            ret = func(arg1, arg2, arg3)
        elapsed_time = time.time() - start
        # print("Spent time = %.3f(msec)" % (elapsed_time*1000))
        return ret, elapsed_time

    def testReadWritePosition(self, dyn, DXL_IDs):
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        dyn.writeOperatingMode(DXL_IDs, [3] * len(DXL_IDs))
        dyn.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

        start = time.time()
        dyn.writeGoalPosition(DXL_IDs, [90 / 0.088] * len(DXL_IDs))
        elapsed_time = time.time() - start
        print("elapsed_time(write):{0}".format(elapsed_time * 1000) + "[msec]")
        time.sleep(1.0)
        start = time.time()
        pos = dyn.readPresentPosition(DXL_IDs)
        elapsed_time = time.time() - start
        print(pos)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeGoalPosition(DXL_IDs, [0] * len(DXL_IDs))
        time.sleep(1.0)
        start = time.time()
        pos = dyn.readPresentPosition(DXL_IDs)
        elapsed_time = time.time() - start
        print(pos)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        # ret, elapsed_time = timeMesurement(dyn.writeGoalPosition, 2, DXL_IDs, [0,0])
        # print("Spent time = %.3f(msec)" % (elapsed_time*1000))

    def testReadWriteVelocity(self, dyn, DXL_IDs):
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        dyn.writeOperatingMode(DXL_IDs, [1] * len(DXL_IDs))
        dyn.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

        start = time.time()
        dyn.writeGoalVelocity(DXL_IDs, [200] * len(DXL_IDs))
        elapsed_time = time.time() - start
        print("elapsed_time(write):{0}".format(elapsed_time * 1000) + "[msec]")
        time.sleep(1.0)
        start = time.time()
        vel = dyn.readPresentVelocity(DXL_IDs)
        elapsed_time = time.time() - start
        print(vel)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeGoalVelocity(DXL_IDs, [0] * len(DXL_IDs))
        time.sleep(1.0)
        start = time.time()
        vel = dyn.readPresentVelocity(DXL_IDs)
        elapsed_time = time.time() - start
        print(vel)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))

    def testReadWriteCurrent(self, dyn, DXL_IDs):
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
        dyn.writeOperatingMode(DXL_IDs, [0] * len(DXL_IDs))
        dyn.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))

        start = time.time()
        dyn.writeGoalCurrent(DXL_IDs, [-100] * len(DXL_IDs))
        elapsed_time = time.time() - start
        print("elapsed_time:{0}".format(elapsed_time * 1000) + "[msec]")
        time.sleep(1.0)
        start = time.time()
        vel = dyn.readPresentCurrent(DXL_IDs)
        elapsed_time = time.time() - start
        print(vel)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeGoalCurrent(DXL_IDs, [100] * len(DXL_IDs))
        time.sleep(1.0)
        start = time.time()
        vel = dyn.readPresentCurrent(DXL_IDs)
        elapsed_time = time.time() - start
        print(vel)
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")
        dyn.writeGoalCurrent(DXL_IDs, [0] * len(DXL_IDs))
        time.sleep(0.5)
        dyn.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))

    def testReadSettings(self, dyn, DXL_IDs):
        start = time.time()
        print(dyn.readReturnDelayTime(DXL_IDs))
        print(dyn.readDriveMode(DXL_IDs))
        print(dyn.readOperatingMode(DXL_IDs))
        print(dyn.readTorqueEnable(DXL_IDs))
        print(dyn.readLED(DXL_IDs))
        print(dyn.readStatusReturnLevel(DXL_IDs))
        # print(dyn.readPresentDGain(DXL_IDs))
        elapsed_time = time.time() - start
        self.elapsed_time = elapsed_time
        print("elapsed_time(read):{0}".format(elapsed_time * 1000) + "[msec]")


if __name__ == "__main__":
    DXL_IDs = [1, 2]
    BAUDRATE = 1e6  # Dynamixel default baudrate : 57600
    # Check which port is being used on your controller
    # DEVICENAME                  = 'COM3' # for Win
    # DEVICENAME                  = '/dev/tty.usbserial-A906DIB5' # for macOS
    DEVICENAME = "/dev/tty.usbserial-FT6Z5XDE"  # for macOS
    dyn = Dynamixel(DEVICENAME, BAUDRATE)
    dyn.setRecommendedValue(DXL_IDs)

    test_dynamixel = Test_Dynamixel()
    test_dynamixel.testReadWritePosition(dyn, DXL_IDs)
    test_dynamixel.testReadWriteVelocity(dyn, DXL_IDs)
    test_dynamixel.testReadWriteCurrent(dyn, DXL_IDs)
    test_dynamixel.testReadSettings(dyn, DXL_IDs)
