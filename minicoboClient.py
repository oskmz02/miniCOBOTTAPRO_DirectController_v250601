import struct
import socket


# Tailescale内
TAILESCALE_DEVPC = "100.105.147.107"
TAILESCALE_FRANKAHP1 = "100.80.3.23"
TAILESCALE_FRANKAHP2 = "100.91.2.34"
TAILESCALE_QUEST = "100.104.4.54"

ADDR_IP = TAILESCALE_DEVPC
ADDR_TX = 22225
ADDR_RX = 22224

ADDR_QUEST_IP = TAILESCALE_QUEST
ADDR_QUEST_TX = 8051
ADDR_QUEST_RX = 8050
# C++構造体のフォーマット定義
STRUCT_FORMAT = "<6i2I"  # 6つのint32_t + 2つのuint32_t


class Axis6i:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.a = 0
        self.b = 0
        self.c = 0


# 値を格納するToRobotクラス
class ToRobot:
    def __init__(self):
        # 初期値を定義
        # self.pos = [0] * 6  # x, y, z, a, b, c
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
        return struct.pack(STRUCT_FORMAT, *pos, self.cmnd, self.pad_)

    @classmethod
    def from_bytes(cls, data):
        """
        バイト列から構造体を生成する
        """
        unpacked_data = struct.unpack(STRUCT_FORMAT, data)
        obj = list(unpacked_data[:6]) + [unpacked_data[6], unpacked_data[7]]
        return obj


class CSocket:
    def __init__(self):
        self.csocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.csocket.bind(("", ADDR_RX))

    def SendTo(self, txbuf):
        self.csocket.sendto(txbuf, (ADDR_IP, ADDR_TX))
        print(f"send")


class QSocket:
    def __init__(self):
        self.qsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qsocket.bind(("", ADDR_QUEST_RX))

    def SendToQuest(self, toquest):
        """
        ToQuest インスタンスをバイト列に変換して送信
        """
        if len(toquest) != 24:
            print(f"Warning: toquest size = {len(toquest)} (expected 24)")
        else:
            self.qsocket.sendto(toquest, (ADDR_QUEST_IP, ADDR_QUEST_TX))
            print(f"Sent to Quest: {len(toquest)} bytes")


class QSocketQuat:
    def __init__(self):
        self.qsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.qsocket.bind(("", ADDR_QUEST_RX))

    def SendToQuest(self, toquest):
        """
        ToQuest インスタンスをバイト列に変換して送信
        """
        if len(toquest) != 96:
            print(f"Warning: toquest size = {len(toquest)} (expected 96)")
        else:
            self.qsocket.sendto(toquest, (ADDR_QUEST_IP, ADDR_QUEST_TX))
            print(f"Sent to Quest: {len(toquest)} bytes")
