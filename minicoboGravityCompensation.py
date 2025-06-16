import numpy as np
from Dynamixel import Dynamixel
from minicoboParameter import MiniCoboParam


class MiniCoboGravCom:
    def __init__(self, dxl: Dynamixel, DXL_IDs):
        mcp = MiniCoboParam(dxl, DXL_IDs)
        self._length_01 = mcp.length_01
        self._length_12 = mcp.length_12
        self._length_23 = mcp.length_23
        self._length_34 = mcp.length_34
        self._length_45 = mcp.length_45
        self._length_56 = mcp.length_56
        self._length_6end = mcp.length_6end
        self._length_5end = mcp.length_5end
        self._length_35 = mcp.length_35
        self._length_36 = mcp.length_36

        self._weight_j23 = mcp.weight_j23
        self._weight_j36 = mcp.weight_j36

        self._tp_grav_j23 = mcp.tp_grav_j23
        self._tp_grav_j36 = mcp.tp_grav_j36
        self._tp_grav_j3 = mcp.tp_grav_j3

    def gravComSimple(self, theata_rad):
        """
        簡易重力補償 : theata_radから重力によるトルクを算出
        """
        self._theata_rad = theata_rad

        # 重力補償用角度算出
        theata_j2_grav = (np.pi / 2) - self._theata_rad[1]
        theata_j3_grav = self._theata_rad[2] - self._theata_rad[1]

        # 重力補償計算
        tq_grav_j2 = -self._weight_j36 * (
            self._tp_grav_j36 * (self._length_36 * np.cos(theata_j3_grav))
            + self._tp_grav_j23 * (self._length_23[2][0] * np.cos(theata_j2_grav))
        ) + -self._weight_j23 * self._tp_grav_j23 * self._length_23[2][0] * np.cos(
            theata_j2_grav
        )
        tq_grav_j3 = (
            self._tp_grav_j3
            * self._weight_j36
            * self._length_36
            * np.cos(theata_j3_grav)
        )
        print(f"tq_j2_grav:{tq_grav_j2}, tq_j3_grav:{tq_grav_j3}")
        tq_gravcom = np.array([0, tq_grav_j2, tq_grav_j3, 0, 0, 0]).astype(np.int16)
        return tq_gravcom
