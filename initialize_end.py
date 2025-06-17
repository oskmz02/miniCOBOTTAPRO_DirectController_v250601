import os
import time
import numpy as np

from Dynamixel import Dynamixel


class InitializeEnd:
    """初期化＆終了処理クラス
    dxl: DynamixelSDKクラスオブジェクト
    DXL_IDs: Dynamixel ID (list)
    """

    def __init__(self, dxl: Dynamixel, DXL_IDs):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs

    # 初期化要処理
    # 組み立て時Dynamixelの2048(180°)の位置を初期姿勢としたときのイニシャライズ
    def initializepostion(self):
        # return で初期位置init_goal_pos
        try:
            init_goal_pos = np.zeros(len(self.DXL_IDs))
            for i in range(len(self.DXL_IDs)):
                init_goal_pos[i] = 2048
            PROFILEACCELERATION = np.array([36, 36, 36, 48, 48, 48])
            PROFILEVELOCITY = np.array([18, 18, 32, 36, 36, 24])
            self.dxl.writeProfileAcceleration(self.DXL_IDs, PROFILEACCELERATION)
            self.dxl.writeProfileVelocity(self.DXL_IDs, PROFILEVELOCITY)
            POSDGAIN = np.array([7200, 9600, 4600, 4800, 600, 600])
            POSIGAIN = np.array([300, 300, 144, 800, 400, 400])
            POSPGAIN = np.array([800, 800, 800, 1420, 600, 600])
            self.dxl.writePositionDGain(self.DXL_IDs, POSDGAIN)
            self.dxl.writePositionIGain(self.DXL_IDs, POSIGAIN)
            self.dxl.writePositionPGain(self.DXL_IDs, POSPGAIN)

            for i in range(1):  # 100回分解能で終了地点に向かう
                print(f"\n\nーーーー 初期化中 ーーーー")
                self.dxl.writeGoalPosition(self.DXL_IDs, init_goal_pos)
                time.sleep(3)
                # os.system("clear")
            for i in range(5):
                # os.system("clear")
                print(f"\n\nーーーー {5-i}秒後に開始します ーーーー")
                time.sleep(1)
            # os.system("clear")
            return init_goal_pos
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [1] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 初期化強制終了 |-----------")

    # 組み立て時Dynamixelの0位置を初期姿勢とした場合のイニシャライズ
    def initializepostion_0(self, pos_ref):
        # return で初期位置init_goal_pos
        try:
            init_goal_pos = np.zeros(len(self.DXL_IDs))
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            sum_goal_pos = pos_ref
            for i in range(len(self.DXL_IDs)):
                if pos_ref[i] >= 2500:
                    init_goal_pos[i] = 4096
                else:
                    init_goal_pos[i] = 0
                diff_goal_pos[i] = init_goal_pos[i] - pos_ref[i]
            PROFILEACCELERATION = np.array([60, 60, 60, 100, 100, 100])
            PROFILEVELOCITY = np.array([24, 24, 24, 30, 30, 30])
            self.dxl.writeProfileAcceleration(self.DXL_IDs, PROFILEACCELERATION)
            self.dxl.writeProfileVelocity(self.DXL_IDs, PROFILEVELOCITY)
            print(f"aaaaaaaaaaaaaaaa")

            for i in range(1):  # 1回分解能で終了地点に向かう
                print(f"\n\nーーーー 初期化中 ーーーー")
                sum_goal_pos += diff_goal_pos
                self.dxl.writeGoalPosition(self.DXL_IDs, sum_goal_pos)
                time.sleep(5)
                # os.system("clear")
            for i in range(3):
                # os.system("clear")
                print(f"\n\nーーーー {3-i}秒後に開始します ーーーー")
                time.sleep(1)
            # os.system("clear")
            return init_goal_pos
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [0] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 初期化強制終了 |-----------")

    def initializepostion100step(self, pos_ref):
        # return で初期位置init_goal_pos
        try:
            init_goal_pos = np.zeros(len(self.DXL_IDs))
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            sum_goal_pos = pos_ref
            for i in range(len(self.DXL_IDs)):
                if pos_ref[i] >= 2500:
                    init_goal_pos[i] = 4096
                else:
                    init_goal_pos[i] = 0
                diff_goal_pos[i] = (init_goal_pos[i] - pos_ref[i]) / 100

            for i in range(100):  # 100回分解能で終了地点に向かう
                print(f"\n\nーーーー 初期化中 ーーーー")
                sum_goal_pos += diff_goal_pos
                self.dxl.writeGoalPosition(self.DXL_IDs, sum_goal_pos)
                time.sleep(0.02)
                # os.system("clear")
            for i in range(3):
                # os.system("clear")
                print(f"\n\nーーーー {3-i}秒後に開始します ーーーー")
                time.sleep(1)
            # os.system("clear")
            return init_goal_pos
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [0] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 初期化強制終了 |-----------")

    def initializepostion_nh(self, pos_ref):
        # noHoming
        try:
            init_goal_pos = np.zeros(len(self.DXL_IDs))
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            sum_goal_pos = pos_ref
            for i in range(len(self.DXL_IDs)):
                if pos_ref[i] >= 2500:
                    init_goal_pos[i] = 4126
                    if i == 3:
                        init_goal_pos[3] -= 40
                    elif i == 4:
                        init_goal_pos[4] -= 30
                else:
                    init_goal_pos[i] = -30
                    if i == 3:
                        init_goal_pos[3] += 40
                    elif i == 4:
                        init_goal_pos[4] += 30
                diff_goal_pos[i] = (init_goal_pos[i] - pos_ref[i]) / 100
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [0] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 初期化強制終了 |-----------")

    # 終了処理用関数
    # 組み立て時Dynamixelの2048(180°)の位置を初期姿勢としたときの終了処理
    def endprocess(self, init_goal_pos, pos_output):
        try:
            PROFILEACCELERATION = np.array([36, 36, 36, 48, 48, 48])
            PROFILEVELOCITY = np.array([18, 18, 18, 36, 36, 24])
            self.dxl.writeProfileAcceleration(self.DXL_IDs, PROFILEACCELERATION)
            self.dxl.writeProfileVelocity(self.DXL_IDs, PROFILEVELOCITY)
            POSDGAIN = np.array([3600, 9600, 7200, 1200, 600, 600])
            POSIGAIN = np.array([100, 800, 800, 400, 250, 150])
            POSPGAIN = np.array([800, 420, 420, 300, 400, 400])
            self.dxl.writePositionDGain(self.DXL_IDs, POSDGAIN)
            self.dxl.writePositionIGain(self.DXL_IDs, POSIGAIN)
            self.dxl.writePositionPGain(self.DXL_IDs, POSPGAIN)
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            end_goal_pos = np.zeros(len(self.DXL_IDs))
            diff_goal_pos = np.array(
                [2048.0, 1575.0, 1235.0, 2100.0, 1272.0, 2048.0]
            )  # 絶対位置指定
            for i in range(1):  # 終了地点に向かう
                print(f"\n\nーーーー 終了処理中 ーーーー")
                end_goal_pos += diff_goal_pos
                self.dxl.writeGoalPosition(self.DXL_IDs, end_goal_pos)
                print(end_goal_pos)
                time.sleep(5)
                print(f"\n[MINICOBO] Push START/STOP BUTTON to start operation")
                # os.system("clear")
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [1] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 終了時強制終了 |-----------")

    def endprocess_100step(self, init_goal_pos, pos_output):
        try:
            PROFILEACCELERATION = np.array([100, 100, 100, 100, 100, 100])
            PROFILEVELOCITY = np.array([24, 24, 24, 30, 30, 30])
            self.dxl.writeProfileAcceleration(self.DXL_IDs, PROFILEACCELERATION)
            self.dxl.writeProfileVelocity(self.DXL_IDs, PROFILEVELOCITY)
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            sum_goal_pos = pos_output.astype(np.float64)
            diff_goal_pos = (
                np.array([1700.0, 1575.0, 1235.0, 2100.0, 1272.0, 2048.0])
                - (pos_output.astype(np.float64))  # 絶対位置指定
            ) / 100
            for i in range(100):  # 100回分解能で終了地点に向かう
                print(f"\n\nーーーー 終了処理中 ーーーー")
                sum_goal_pos += diff_goal_pos
                self.dxl.writeGoalPosition(self.DXL_IDs, sum_goal_pos)
                print(sum_goal_pos)
                time.sleep(0.03)
                # os.system("clear")
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [0] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 終了時強制終了 |-----------")

    # 組み立て時Dynamixelの0位置を初期姿勢とした場合の終了処理
    def endprocess_0(self, init_goal_pos, pos_output):
        try:
            diff_goal_pos = np.zeros(len(self.DXL_IDs))
            sum_goal_pos = pos_output.astype(np.float64)
            diff_goal_pos = (
                np.array([0.0, 678.0, -286.0, 1044.0, 0.0, 0.0])
                - (pos_output.astype(np.float64) - init_goal_pos.astype(np.float64))
            ) / 100
            for i in range(100):  # 100回分解能で終了地点に向かう
                print(f"\n\nーーーー 終了処理中 ーーーー")
                sum_goal_pos += diff_goal_pos
                self.dxl.writeGoalPosition(self.DXL_IDs, sum_goal_pos)
                print(sum_goal_pos)
                time.sleep(0.05)
                # os.system("clear")
        except KeyboardInterrupt:
            self.dxl.writeTorqueEnable(
                self.DXL_IDs, [0] * len(self.DXL_IDs)
            )  # Torque off
            print("-----------| 終了時強制終了 |-----------")
