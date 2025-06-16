import numpy as np


class CoboKinematic2:
    def __init__(self, dxl, DXL_IDs):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs
        # COBIT各種パラメーター
        # アーム長さ
        self.length_01 = np.array([[0.0], [0.0], [40.0]])  # [mm]
        self.length_12 = np.array([[0.0], [40.3], [44.0]])
        self.length_23 = np.array([[0.0], [11.8], [204.0]])
        self.length_34 = np.array([[120.0], [40.5], [0.0]])
        self.length_45 = np.array([[36.0], [12.0], [0.0]])
        self.length_56 = np.array([[0.0], [48.0], [32.5]])
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

    def tsZ(self, theata_rad: float, length):
        """Z軸回り同次変換行列
        Args:
            theata_rad (float)
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [np.cos(theata_rad), -np.sin(theata_rad), 0, length[0][0]],
                [np.sin(theata_rad), np.cos(theata_rad), 0, length[1][0]],
                [0, 0, 1, length[2][0]],
                [0, 0, 0, 1],
            ]
        )

    def tsY(self, theata_rad: float, length):
        """Y軸回りの同次変換行列
        Args:
            theata_rad (float)
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [np.cos(theata_rad), 0, np.sin(theata_rad), length[0][0]],
                [0, 1, 0, length[1][0]],
                [-np.sin(theata_rad), 0, np.cos(theata_rad), length[2][0]],
                [0, 0, 0, 1],
            ]
        )

    def tsX(self, theata_rad: float, length):
        """X軸回りの同次変換行列
        Args:
            theata_rad (float)
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [1, 0, 0, length[0][0]],
                [0, np.cos(theata_rad), -np.sin(theata_rad), length[1][0]],
                [0, np.sin(theata_rad), np.cos(theata_rad), length[2][0]],
                [0, 0, 0, 1],
            ]
        )

    def rotationRight(self):
        return np.array(
            [
                [1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1],
            ]
        )

    def rotationXPi(self):
        """X軸回りに180°回転
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )

    def rotationYPi(self):
        """Y軸回りに180°回転
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [-1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )

    def rotationZPi(self):
        """Z軸回りに180°回転
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

    def MirrorX(self):
        return np.array(
            [
                [-1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

    def MirrorY(self):
        return np.array(
            [
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

    def MirrorZ(self):
        return np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )

    def shiftY(self, length):
        """Y軸方向の平行移動
        Args:
            length (float)
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [1, 0, 0, length[0][0]],
                [0, 1, 0, length[1][0]],
                [0, 0, 1, length[2][0]],
                [0, 0, 0, 1],
            ]
        )

    def shiftZ(self, length):
        """Z軸方向の平行移動
        Args:
            length (float)
        Returns:
            _type_: matrix
        """
        return np.array(
            [
                [1, 0, 0, length[0][0]],
                [0, 1, 0, length[1][0]],
                [0, 0, 1, length[2][0]],
                [0, 0, 0, 1],
            ]
        )

    def extractRotationMatrix(self, Ts):
        rotation = Ts[:3, :3]
        return rotation

    def extractVetorMatrix(self, Ts):
        pos = Ts[:3, 3]
        return pos

    def toEulerAngleZYX(self, rot):
        """
        ZYX順序で回転行列をEuler角に変換する。
        :param rot: 3x3の回転行列
        :return: Z, Y, X軸の回転角の配列
        """
        # rot = -rot
        sy = -rot[2, 0]
        unlocked = np.abs(sy) < 0.99999  # ジンバルロックの検出

        if unlocked:
            rx = np.arctan2(rot[2, 1], rot[2, 2])  # X軸周りの回転
            ry = np.arcsin(sy)  # Y軸周りの回転
            rz = np.arctan2(rot[1, 0], rot[0, 0])  # Z軸周りの回転
        else:
            rx = 0
            ry = np.arcsin(sy)
            rz = np.arctan2(-rot[0, 1], rot[1, 1])
        return np.array([rx, ry, rz])

    def toEulerAngleXYZ(self, rot):
        """
        XYZ順序で回転行列をEuler角に変換する。
        :param rot: 3x3の回転行列
        :return: X, Y, Z軸の回転角の配列
        """
        sy = rot[0, 2]
        unlocked = np.abs(sy) < 0.99999  # ジンバルロックの検出

        if unlocked:
            rx = np.arctan2(-rot[1, 2], rot[2, 2])  # X軸周りの回転
            ry = np.arcsin(sy)  # Y軸周りの回転
            rz = np.arctan2(-rot[0, 1], rot[0, 0])  # Z軸周りの回転
        else:
            rx = np.arctan2(rot[2, 1], rot[2, 2])
            ry = np.arcsin(sy)
            rz = 0
        return np.array([rx, ry, rz])

    def fowardKinematic(self, length, theata_rad):
        Ts = np.eye(4, 4)
        for i in range(len(self.DXL_IDs)):
            if i == 0:
                Ts_rot = self.tsZ(theata_rad[i], length[i])
            elif i == 1:
                Ts_rot = self.tsY(theata_rad[i], length[i])
            elif i == 2:
                Ts_rot = self.tsY(theata_rad[i], length[i])
            elif i == 3:
                Ts_rot = self.tsX(theata_rad[i], length[i])
            elif i == 4:
                Ts_rot = self.tsY(theata_rad[i], length[i])
            elif i == 5:
                Ts_rot = self.tsZ(theata_rad[i], length[i])
            Ts = Ts.dot(Ts_rot)
            if i == 1 or i == 2:
                Ts = np.dot(Ts, self.rotationZPi())
            if i == 2 or i == 3:
                Ts = np.dot(Ts, self.rotationXPi())
            if i == 4:
                Ts = np.dot(Ts, self.rotationYPi())
        Ts = np.dot(Ts, self.shiftY(length[6]))

        return Ts

    def inverseKinematic(self, desired_T, theata_initial=None):
        """
        逆運動学: 目標エンドエフェクタの同次変換行列から関節角度を算出する
        Args:
            desired_T (4x4 array): 目標のエンドエフェクタ姿勢
            theata_initial (list or None): 初期角度推定値（反復法利用時など）
        Returns:
            theata (ndarray): 関節角度 [q1,...,q6]
        """
        # 1) 目標位置・回転を抽出
        Rd = self.extractRotationMatrix(desired_T)
        pd = self.extractVetorMatrix(desired_T)

        # 2) リンク末端オフセットを引いて手首中心位置を計算
        d6 = self.length[6].flatten()  # 最終オフセット [0,0,d6]
        # ツールフレームZ軸方向（ShiftYでZ方向）に沿ったオフセット
        pwc = pd - Rd.dot(d6)

        # 3) 関節1の角度 q1
        q1 = np.arctan2(pwc[1], pwc[0])

        # 4) 関節2,3 の平面ジオメトリで解く
        # ベース高さオフセット
        d1 = self.length[0][2, 0]
        # リンク長 a2, a3
        a2 = np.linalg.norm(self.length[1])
        a3 = np.linalg.norm(self.length[2])
        # 平面投影距離
        r = np.hypot(pwc[0], pwc[1])
        s = pwc[2] - d1
        # 余弦定理
        cos_q3 = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        q3 = np.arccos(cos_q3)
        # q2
        alpha = np.arctan2(s, r)
        beta = np.arctan2(a3 * np.sin(q3), a2 + a3 * np.cos(q3))
        q2 = alpha - beta

        # 5) 手首回転(4-6軸)を求める
        # 前腕までの変換行列 T0_3
        thetas_123 = [q1, q2, q3]
        Ts = np.eye(4)
        for i, q in enumerate(thetas_123):
            if i == 0:
                T = self.tsZ(q, self.length[i])
            elif i == 1:
                T = self.tsY(q, self.length[i])
            else:
                T = self.tsY(q, self.length[i])
            Ts = Ts.dot(T)
            if i == 1 or i == 2:
                Ts = Ts.dot(self.rotationZPi())
            if i == 2:
                Ts = Ts.dot(self.rotationXPi())
        # T0_3 の回転部分
        R0_3 = Ts[:3, :3]

        # 目標の手首回転行列 R3_6
        R3_6 = R0_3.T.dot(Rd)
        # Euler ZYX で分解
        q4, q5, q6 = self.toEulerAngleZYX(R3_6)

        theata = np.array([q1, q2, q3, q4, q5, q6])
        return theata
