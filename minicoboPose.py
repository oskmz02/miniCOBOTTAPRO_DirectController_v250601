import numpy as np
import struct
import copy as cp
from Dynamixel import Dynamixel
from minicoboParameter import MiniCoboParam


class Axis6i:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.a = 0
        self.b = 0
        self.c = 0


class Jnt6i:
    def __init__(self):
        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j5 = 0
        self.j6 = 0


class Quaternion:
    def __init__(self):
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 1


class ToRobot:
    def __init__(self):
        self.pos = Axis6i()
        self.cmnd = 0
        self.pad_ = 0

    def OnSendData(self):
        """
        構造体をバイト列に変換する
        """
        # Axis6iの各値をバイナリに変換
        pos = [
            int(self.pos.x),
            int(self.pos.y),
            int(self.pos.z),
            int(self.pos.a),
            int(self.pos.b),
            int(self.pos.c),
        ]
        # 構造体全体をパック
        return struct.pack("<6i2I", *pos, self.cmnd, self.pad_)

    @classmethod
    def from_bytes(cls, data):
        """
        バイト列から構造体を生成する
        """
        unpacked_data = struct.unpack("<6i2I", data)
        obj = list(unpacked_data[:6]) + [unpacked_data[6], unpacked_data[7]]
        return obj


class ToQuest:
    def __init__(self):
        self.jnt = Jnt6i()
        self._lastPacket = b"\x00" * 24  # 例外初期化

    def OnSendData(self):
        data = [
            int(self.jnt.j1),
            int(self.jnt.j2),
            int(self.jnt.j3),
            int(self.jnt.j4),
            int(self.jnt.j5),
            int(self.jnt.j6),
        ]
        try:
            packet = struct.pack("<6i", *data)  # 6 個の int32 → 24 バイト
        except struct.error as e:
            print(f"ToQuest.OnSendData pack error: {e}; using dummy packet.")
            packet = self._lastPacket
        else:
            self._lastPacket = packet
        return packet


class ToQuestQuat:
    def __init__(self):
        self.qs = [Quaternion() for _ in range(6)]

    def OnSendData(self):
        data = []
        for q in self.qs:
            data.extend([int(q.x), int(q.y), int(q.z), int(q.w)])
        try:
            packet = struct.pack("<24i", *data)  # 24 個の int32 → 96 バイト
        except struct.error as e:
            print(f"ToQuest.OnSendData pack error: {e}; using dummy packet.")
            packet = self._lastPacket
            # もし前回がないならゼロ埋めにしたい場合は下記を使う
            # packet = struct.pack("<24i", *([0]*24))
        else:
            self._lastPacket = packet
        return packet


class MiniCoboPose:
    def __init__(self, dxl: Dynamixel, DXL_IDs):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs
        mcp = MiniCoboParam(dxl, DXL_IDs)
        self._length = mcp.length
        self._sp = mcp.sp
        self.knmtc = CoboKinematic(self.dxl, self.DXL_IDs)
        self.startpos = Axis6i()
        self.startrobot = Axis6i()
        self.pos = Axis6i()
        self.rel = Axis6i()
        self.torobot = ToRobot()
        self.send = ToRobot()
        self.Tsl_abs = None
        self.Tsl_rel = None
        self.rotl_rel = np.empty((len(self.DXL_IDs), 3, 3))
        self.toquest = ToQuest()
        self.toquest_quat = ToQuestQuat()

    def setPose(self, theata, option):
        """
        角度(Dynamixel Unit)からminiCOBOTTAPROの各運動学情報を得る
        option
        1 : ToRobot構造体までを計算.
        2 : 1 + Quest送信用データも取得.
        3 : QuestにQuaternion送る用(実装済み未使用)
        """
        self.theata = theata
        self.deg = (np.floor(((360 / 4096) * self.theata) * 100)) / 100
        self.rad = np.round((self.theata * (np.pi / 2048)), 12)

        if option == 1:
            self.getFK()
            self.getRotMat()
            self.getVecMat()
            self.getEuler()
            self.torobot = self.getToRobot()  # ToRobot構造体に変換

        elif option == 2:
            self.getFK()
            self.getRotMat()
            self.getVecMat()
            self.getEuler()
            self.toquest = self.getToQuest()
            self.torobot = self.getToRobot()

        elif option == 3:
            self.getFK()
            self.getFKJnt()
            self.getRotMat()
            self.getVecMat()
            self.getEuler()
            self.getQuat(self.rotl_rel)
            self.toquest_quat = self.getToQuestQuat()
            self.torobot = self.getToRobot()

    def getFK(self):
        """
        同次変換行列の取得
        """
        self.Ts = self.knmtc.fowardKinematic(self._length, self.rad)
        self.Tsl_abs = self.knmtc.Tsl_abs

    def getFKJnt(self):
        """
        各軸ごとの同次変換行列の取得
        """
        self.Tsl_rel = self.knmtc.fowardKinematicJnt(self._length, self.rad)

    def getRotMat(self):
        """
        同次変換行列から回転行列を抽出
        """
        self.rot = self.knmtc.extractRotationMatrix(self.Ts)
        if self.Tsl_rel is not None and self.Tsl_rel.size > 0:
            for i in range(len(self.DXL_IDs)):
                self.rotl_rel[i] = self.knmtc.extractRotationMatrix(self.Tsl_rel[i])
            # print(f"rotl_rel : {self.rotl_rel}")

    def getVecMat(self):
        """
        同次変換行列からデカルト直交位置座標を示す行列を抽出
        """
        self.position = self.knmtc.extractVetorMatrix(self.Ts)

    def getEuler(self):
        """
        回転行列をZYXオイラー角に変換
        """
        self.euler = self.knmtc.toEulerAngleZYX(self.rot)

    def getQuat(self, data):
        """
        dataの形式に合わせてクォータニオンに変換
        """
        data = np.array(data)
        if data.shape == (3, 3):
            self.quat = self.knmtc.toQuaternion(data, 1)

        elif data.ndim == 3 and data.shape[1:] == (3, 3):
            self.quatl_rel = np.empty((data.shape[0], 4))
            for i in range(data.shape[0]):
                self.quatl_rel[i] = self.knmtc.toQuaternion(data[i], 1)

        elif data.shape == (3,):
            self.quat = self.knmtc.toQuaternion(data, 2)

        else:
            self.quat = [0, 0, 0, 0]
            print(f"Quaternionに変換できません")

    def getToQuest(self):
        send = ToQuest()
        # 6軸分
        send.jnt.j1 = -self.deg[0] * 1e6  # intで送信のためスケーリング
        send.jnt.j2 = -self.deg[1] * 1e6  # なんか回転方向反転してたので-
        send.jnt.j3 = -self.deg[2] * 1e6
        send.jnt.j4 = -self.deg[3] * 1e6
        send.jnt.j5 = -self.deg[4] * 1e6
        send.jnt.j6 = -self.deg[5] * 1e6
        return send

    def getToQuestQuat(self):
        send = ToQuestQuat()
        # 6軸分
        for i in range(6):
            # クォータニオン取得 (w, x, y, z の順)
            w, x, y, z = self.quatl_rel[i]
            # Unity用に (x, y, z, w) の順に並べ替え，スケーリング
            send.qs[i].x = int(z * 1e9)
            send.qs[i].y = int(-y * 1e9)
            send.qs[i].z = int(x * 1e9)
            send.qs[i].w = int(w * 1e9)
            print(f"toquest : {send.qs[i].x, send.qs[i].y, send.qs[i].z, send.qs[i].w}")
        return send

    def getToRobot(self):
        # 初期状態での姿勢情報の取得
        if self.send.pad_ == 0:
            self.startpos.x = self.position[0] * self._sp
            self.startpos.y = self.position[1] * self._sp
            self.startpos.z = self.position[2] * self._sp
            self.startpos.a = self.euler[0] * (180 / np.pi)
            self.startpos.b = self.euler[1] * (180 / np.pi)
            self.startpos.c = self.euler[2] * (180 / np.pi)
            # 本来はロボットのエンコーダ値を取得したい
            self.startrobot.x = int(self.position[0] * 1e6 * self._sp)
            self.startrobot.y = int(self.position[1] * 1e6 * self._sp)
            self.startrobot.z = int(self.position[2] * 1e6 * self._sp)
            self.startrobot.a = int(self.euler[0] * 1e6 * (180 / np.pi))
            self.startrobot.b = int(self.euler[1] * 1e6 * (180 / np.pi))
            self.startrobot.c = int(self.euler[2] * 1e6 * (180 / np.pi))

        # ToRobot構造体に代入
        self.pos.x = self.position[0] * self._sp
        self.pos.y = self.position[1] * self._sp
        self.pos.z = self.position[2] * self._sp
        self.pos.a = self.euler[0] * (180 / np.pi)
        self.pos.b = self.euler[1] * (180 / np.pi)
        self.pos.c = self.euler[2] * (180 / np.pi)

        print(f"pos.x:{np.round(self.pos.x, 3)}")
        print(f"pos.y:{np.round(self.pos.y, 3)}")
        print(f"pos.z:{np.round(self.pos.z, 3)}")
        print(f"pos.a:{np.round(self.pos.a, 3)}")
        print(f"pos.b:{np.round(self.pos.b, 3)}")
        print(f"pos.c:{np.round(self.pos.c, 3)}")

        self.rel.x = int((self.pos.x - self.startpos.x) * 1e6)
        self.rel.y = int((self.pos.y - self.startpos.y) * 1e6)
        self.rel.z = int((self.pos.z - self.startpos.z) * 1e6)
        self.rel.a = int((self.pos.a - self.startpos.a) * 1e6)
        self.rel.b = int((self.pos.b - self.startpos.b) * 1e6)
        self.rel.c = int((self.pos.c - self.startpos.c) * 1e6)

        self.send.pos.x = self.startrobot.x + self.rel.x
        self.send.pos.y = self.startrobot.y + self.rel.y
        self.send.pos.z = self.startrobot.z + self.rel.z
        self.send.pos.a = self.startrobot.a + self.rel.a
        self.send.pos.b = self.startrobot.b + self.rel.b
        self.send.pos.c = self.startrobot.c + self.rel.c
        self.send.cmnd = 2
        self.send.pad_ += 1

        return self.send


class CoboKinematic:
    def __init__(self, dxl, DXL_IDs):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs
        self.Tsl_rel = np.empty((len(self.DXL_IDs), 4, 4))
        self.Tsl_abs = np.empty((len(self.DXL_IDs), 4, 4))
        self.rtXPi = np.array(
            [
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )
        self.rtYPi = np.array(
            [
                [-1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]
        )
        self.rtZPi = np.array(
            [
                [-1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
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

    def toQuaternion(self, data, option):
        """
        クォータニオン(w,x,y,z)に変換
        option:1:回転行列からクォータニオン. :2:オイラー角からクォータニオン．
        """
        if option == 1:
            # data は 3x3 の回転行列 np.ndarray
            m = data
            # 各成分を準備
            px = m[0, 0] - m[1, 1] - m[2, 2] + 1
            py = -m[0, 0] + m[1, 1] - m[2, 2] + 1
            pz = -m[0, 0] - m[1, 1] + m[2, 2] + 1
            pw = m[0, 0] + m[1, 1] + m[2, 2] + 1

            # 最大成分を選択
            selected = np.argmax([px, py, pz, pw])
            # np.argmax は 0:px, 1:py, 2:pz, 3:pw を返す

            # 各ケースごとに quaternion を計算
            if selected == 0:
                x = 0.5 * np.sqrt(px)
                d = 1.0 / (4.0 * x)
                y = (m[1, 0] + m[0, 1]) * d
                z = (m[0, 2] + m[2, 0]) * d
                w = (m[2, 1] - m[1, 2]) * d

            elif selected == 1:
                y = 0.5 * np.sqrt(py)
                d = 1.0 / (4.0 * y)
                x = (m[1, 0] + m[0, 1]) * d
                z = (m[2, 1] + m[1, 2]) * d
                w = (m[0, 2] - m[2, 0]) * d

            elif selected == 2:
                z = 0.5 * np.sqrt(pz)
                d = 1.0 / (4.0 * z)
                x = (m[0, 2] + m[2, 0]) * d
                y = (m[2, 1] + m[1, 2]) * d
                w = (m[1, 0] - m[0, 1]) * d

            elif selected == 3:
                w = 0.5 * np.sqrt(pw)
                d = 1.0 / (4.0 * w)
                x = (m[2, 1] - m[1, 2]) * d
                y = (m[0, 2] - m[2, 0]) * d
                z = (m[1, 0] - m[0, 1]) * d

            else:
                w = 0
                x = 0
                y = 0
                z = 0
                print(f"Quaternion ERROR")
            # w, x, y, z の順で返す
            print(
                f"w,x,y,z : {np.round(w,2), np.round(x,2), np.round(y,2), np.round(z,2)}"
            )
            return (w, x, y, z)
        elif option == 2:
            w = 0
            x = 0
            y = 0
            z = 0
            print(f"Quaternion ERROR, オイラー角からの変換は未実装")
        # w, x, y, z の順で返す
        return (w, x, y, z)

    def fowardKinematic(self, length, theata_rad):
        Ts = np.eye(4, 4)
        self.Tsl_abs = np.empty((len(self.DXL_IDs), 4, 4))
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
            self.Tsl_abs[i] = Ts
            if i == 1 or i == 2:
                Ts = np.dot(Ts, self.rtZPi)
            if i == 2 or i == 3:
                Ts = np.dot(Ts, self.rtXPi)
            if i == 4:
                Ts = np.dot(Ts, self.rtYPi)
        Ts = np.dot(Ts, self.shiftY(length[6]))
        return Ts

    def fowardKinematicJnt(self, length, theata_rad):
        Tsl_rel = np.empty((len(self.DXL_IDs), 4, 4))
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

            Tsl_rel[i] = Ts_rot

            if i == 1 or i == 2:
                Ts_rot = np.dot(Ts_rot, self.rtZPi)
            if i == 2 or i == 3:
                Ts_rot = np.dot(Ts_rot, self.rtXPi)
            if i == 4:
                Ts_rot = np.dot(Ts_rot, self.rtYPi)

        return Tsl_rel
