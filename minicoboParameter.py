import numpy as np
from Dynamixel import Dynamixel


class MiniCoboParam:
    def __init__(self, dxl: Dynamixel, DXL_IDs):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs

        self.tq_threshold = np.array([7, 14, 14, 15, 8, 6])  # 位置用
        self.pos_err = np.zeros(len(self.DXL_IDs))
        self.pos_input = np.zeros(len(self.DXL_IDs))

        self.tq_gravcom = np.zeros(len(self.DXL_IDs))
        self.tq_diff = np.zeros(len(self.DXL_IDs))
        self.tq_diff_prev = np.zeros(len(self.DXL_IDs))

        self.del_t = np.array([0.010] * len(self.DXL_IDs))
        self.alpha = np.array([20.0, 16.0, 11.2, 49.0, 108.0, 121.0])
        self.beta = np.array([4.0, 4.0, 4.0, 5.2, 64.0, 64.5])
        self.gamma_0 = np.array([2.0, 2.0, 2.4, 6.4, 1.2, 1.2])
        self.gamma_1 = np.array([8.0, 9.0, 10.0, 14.0, 10.0, 10.0])
        self.gamma_v = np.zeros(len(self.DXL_IDs))
        self.v_0 = np.array([9.0, 4.0, 6.0, 9.0, 20.0, 32.0])

        self.a_cul = np.zeros(len(self.DXL_IDs))
        self.v_diff = np.zeros(len(self.DXL_IDs))
        self.v_ref = np.zeros(len(self.DXL_IDs))
        self.a_lim = np.array([17060, 17060, 17060, 17060, 17060, 17060])
        self.v_lim = np.array([635, 635, 635, 635, 870, 870])

        # COBIT各種パラメーター
        # アーム長さ
        self.length_01 = np.array([[0.0], [0.0], [40.0]])  # [mm]
        self.length_12 = np.array([[0.0], [40.3], [44.0]])  # ベース座標系では-40.3
        self.length_23 = np.array([[0.0], [11.8], [204.0]])
        self.length_34 = np.array([[120.0], [40.5], [0.0]])
        self.length_45 = np.array([[36.0], [12.0], [0.0]])  # ベース座標系では-12.0
        self.length_56 = np.array([[0.0], [48.0], [32.5]])  # ベース座標系では-48.0
        self.length_6end = np.array([[0.0], [0.0], [64.0 - self.length_56[2][0]]])
        self.length_5end = self.length_56[2][0] + self.length_6end[2][0]
        self.length_35 = self.length_34[0][0] + self.length_45[0][0]
        self.length_36 = self.length_35 + self.length_5end
        self.length = np.array(
            [
                self.length_01,
                self.length_12,
                self.length_23,
                self.length_34,
                self.length_45,
                self.length_56,
                self.length_6end,
            ]
        )

        # アーム重さ(正確でない)
        self.weight_j23 = 230  # j2~j3分の荷重[g]
        self.weight_j36 = 100

        # 簡易重力補償係数
        COVER = [0.00117, 0.00131, 0.00197]
        NOCOVER = [0.0009, 0.0007, 0.0017]
        self.tp_grav_j23 = COVER[0]
        self.tp_grav_j36 = COVER[1]
        self.tp_grav_j3 = COVER[2]

        # スケーリング係数
        self.sp = 1 / 0.4

        # ToRobot構造体形式
        self.torobotformat = "<6i2I"  # 6つのint32_t + 2つのuint32_t
