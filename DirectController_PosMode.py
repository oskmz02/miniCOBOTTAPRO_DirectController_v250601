from initialize_end import InitializeEnd
from except_process import ExceptProcess
from minicoboPose import CoboKinematic
from minicoboClient import ToRobot, Axis6i, CSocket
from lowpass import LowPass

from concurrent.futures import ThreadPoolExecutor
import time
from Dynamixel import Dynamixel, Test_Dynamixel
import os
import numpy as np
import struct
import socket


def __init__():
    DXL_IDs = [1, 2, 3, 4, 5, 6]
    BAUDRATE = 4000000  # 基本変更しなくてよし
    DEVICENAME = "/dev/ttyUSB0"  # 接続されているポート指定
    dxl = Dynamixel(DEVICENAME, BAUDRATE)  # Dynamixelクラスイニシャライズ
    tst = Test_Dynamixel()
    dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
    dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
    dxl.writeOperatingMode(DXL_IDs, [5] * len(DXL_IDs))  # モード5：位置電流制御モード
    dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
    tst.testReadSettings(dxl, DXL_IDs)
    time.sleep(2)
    print(f"data clearning...")
    dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
    tst.testReadSettings(dxl, DXL_IDs)
    time.sleep(1)
    dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
    tst.testReadSettings(dxl, DXL_IDs)

    MAX_RETRY = 5
    retry = 0
    while retry <= MAX_RETRY:
        dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
        tst.testReadSettings(dxl, DXL_IDs)
        init_getdata_result = dxl.getdata_result_array
        tst_time = tst.elapsed_time * 1000  # [msec]
        if 5.5 > tst_time or tst_time > 7.5:
            comtime_statu = False
        else:
            comtime_statu = True

        if all(init_getdata_result) != True:
            print(f"通信失敗あり（{retry+1}回目）。再初期化します。")
            dxl.setRecommendedValue(DXL_IDs)
            tst.testReadSettings(dxl, DXL_IDs)
            retry += 1
            time.sleep(2)
        elif (all(init_getdata_result) == True) and (comtime_statu != True):
            print(f"異常な通信結果です．（{retry+1}回目）。再初期化します。")
            dxl.setRecommendedValue(DXL_IDs)
            tst.testReadSettings(dxl, DXL_IDs)
            retry += 1
            time.sleep(4)
        else:
            global communication_statu
            communication_statu = True
            break

    else:
        print("初期化失敗：通信できないサーボがあります")
        # エラー処理または終了処理
        os._exit(1)

    time.sleep(3)
    dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
    dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
    dxl.writeOperatingMode(DXL_IDs, [5] * len(DXL_IDs))  # モード5：位置電流制御モード
    dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
    tst.testReadSettings(dxl, DXL_IDs)
    print(f"communication result:{dxl.getdata_result_array}")
    print(f"wating...")
    time.sleep(2)

    # print("------| initialize Current based Posision Mode |------")

    time.sleep(3)
    currentBasedPositionDirectControl(dxl, DXL_IDs)  # 位置電流制御モード


# def torqueToPositionReference(dxl:Dynamixel, ):


def currentBasedPositionDirectControl(dxl: Dynamixel, DXL_IDs):
    # tq_threshold = np.array([6, 8, 5, 4, 8, 8])  # 位置用
    tq_threshold = np.array([2, 5, 5, 4, 20, 10])  # 位置電流用
    tp = np.array([0.4, 0.3, 0.3, 0.3, 0.3, 0.4])
    # kp = np.array([0.6, 0.7, 0.7, 0.6, 0.6, 0.5])
    kr = np.array([0.6, 0.6, 0.6, 0.6, 0.5, 0.5])
    kp = np.array([0.3, 0.4, 0.4, 0.3, 0.3, 0.2])
    # kp = np.zeros(len(DXL_IDs))
    pos_ref = np.array(dxl.readPresentPosition(DXL_IDs)).astype(np.float64)
    pos_err = np.zeros(len(DXL_IDs))
    pos_input = np.zeros(len(DXL_IDs))

    pos_init = np.array(dxl.readPresentPosition(DXL_IDs))
    # pos_offset = np.zeros(len(DXL_IDs))

    # COBIT各種パラメーター
    length_01 = np.array([[0.0], [0.0], [40.0]])  # [mm]
    length_12 = np.array([[0.0], [40.3], [44.0]])  # ベース座標系では-40.3
    length_23 = np.array([[0.0], [11.8], [204.0]])
    length_34 = np.array([[120.0], [40.5], [0.0]])
    length_45 = np.array([[36.0], [12.0], [0.0]])  # ベース座標系では-12.0
    length_56 = np.array([[0.0], [48.0], [32.5]])  # ベース座標系では-48.0
    length_6end = np.array([[0.0], [0.0], [64.0 - length_56[2][0]]])
    weight_j23 = 230  # j2~j3分の荷重[g]
    weight_j36 = 100

    length_5end = length_56[2][0] + length_6end[2][0]
    length_35 = length_34[0][0] + length_45[0][0]
    length_36 = length_35 + length_5end

    length = np.array(
        [length_01, length_12, length_23, length_34, length_45, length_56, length_6end]
    )

    # 簡易重力補償係数
    COVER = [0.0012, 0.0011, 0.0022]
    NOCOVER = [0.0009, 0.0007, 0.0017]

    tp_grav_j23 = COVER[0]
    tp_grav_j36 = COVER[1]
    tp_grav_j3 = COVER[2]

    # スケーリング係数
    sp = 1 / 0.4

    # クラス定義
    initializeend = InitializeEnd(dxl, DXL_IDs)
    exceptprocess = ExceptProcess(dxl, DXL_IDs, communication_statu)
    kinematic = CoboKinematic(dxl, DXL_IDs)
    startpos = Axis6i()
    startrobot = Axis6i()
    lowpasspos = Axis6i()
    pos = Axis6i()
    rel = Axis6i()
    send = ToRobot()
    csocket = CSocket()

    pad_ = 0
    com_statu_prev = True

    # kinematic.inv()
    # 位置初期化
    pos_init_goal = initializeend.initializepostion(pos_ref)

    PROFILEACCELERATION = np.array([196, 196, 196, 196, 196, 196])
    PROFILEVELOCITY = np.array([82, 82, 82, 82, 82, 82])
    dxl.writeProfileAcceleration(DXL_IDs, PROFILEACCELERATION)
    dxl.writeProfileVelocity(DXL_IDs, PROFILEVELOCITY)
    print(dxl.readPresentPosition(DXL_IDs) - pos_init_goal)
    POSDGAIN = np.array([800, 1000, 1000, 900, 600, 500])
    POSIGAIN = np.array([160, 720, 720, 720, 400, 150])
    POSPGAIN = np.array([100, 240, 240, 320, 320, 300])
    dxl.writePositionDGain(DXL_IDs, POSDGAIN)
    dxl.writePositionIGain(DXL_IDs, POSIGAIN)
    dxl.writePositionPGain(DXL_IDs, POSPGAIN)

    # 角度とトルクの初期化
    pos_output = np.array(dxl.readPresentPosition(DXL_IDs))
    tq_output = np.array(dxl.readPresentCurrent(DXL_IDs))
    time.sleep(0.009)

    try:
        print("--------------| while start |--------------")
        timer01 = time.time()

        while True:

            start_time = time.time()
            if send.pad_ == 0:
                time_prev = time.time()

            # 受信
            pos_output_rx = np.array(dxl.readPresentPosition(DXL_IDs))
            getdata_result_pos = np.array(dxl.getdata_result_array)

            tq_output_rx = np.array(dxl.readPresentCurrent(DXL_IDs))
            getdata_result_tq = np.array(dxl.getdata_result_array)

            # 毎ループの処理部分
            if (all(getdata_result_pos) != True) or (all(getdata_result_tq) != True):
                # 通信失敗 → カウンタを初期化
                exceptprocess.recovery_wait_counter = 3
                exceptprocess.comErr(False)  # 失敗したらなのでFalse直入れ
                com_statu_prev = False

            elif (
                (com_statu_prev == False)
                and (all(getdata_result_pos) == True)
                and (all(getdata_result_tq) == True)
            ):
                # 通信が直前までダメで今Trueに復帰 → 直後はまだ怪しいのでカウントダウン
                if exceptprocess.recovery_wait_counter > 0:
                    exceptprocess.recovery_wait_counter -= 1
                    print(f"pos_output_rx:{pos_output_rx}")
                    print(f"tq_output_rx:{tq_output_rx}")
                    print("通信復旧後スキップ")
                else:
                    com_statu_prev = True
                    # カウントダウン終了、正常処理開始
                    pos_output = pos_output_rx
                    tq_output = tq_output_rx
                # ここに演算処理を書く

            else:
                com_statu_prev = True
                # ずっと通信が成功している場合
                pos_output = pos_output_rx
                tq_output = tq_output_rx

                # os.system("clear")

                print(pos_init)
                print(pos_init_goal)
                print(f"tq:{tq_output}, pos_output:{pos_output}")

                theata = pos_output - pos_init_goal
                # print(f"theata:{theata}")

                theata_deg = (
                    np.floor(((360 / 4096) * theata) * 100)
                ) / 100  # 小数点第2位までのdegreeの算出

                theata_rad = np.round((theata * (np.pi / 2048)), 12)  # radianの算出

                # 姿勢計算(順運動学)

                Ts = kinematic.fowardKinematic(length, theata_rad)
                rot = kinematic.extractRotationMatrix(Ts)
                position = kinematic.extractVetorMatrix(Ts)
                # print(f"X:{position[0]}, Y:{position[1]}, Z:{position[2]}")
                euler = kinematic.toEulerAngleZYX(rot)
                # print(f"rx:{euler[0]}, ry:{euler[1]}, rz:{euler[2]}")

                # 初期状態での姿勢情報の取得
                if send.pad_ == 0:
                    startpos.x = position[0] * sp
                    startpos.y = position[1] * sp
                    startpos.z = position[2] * sp
                    startpos.a = euler[0] * (180 / np.pi)
                    startpos.b = euler[1] * (180 / np.pi)
                    startpos.c = euler[2] * (180 / np.pi)
                    # 本来はロボットのエンコーダ値を取得したい
                    startrobot.x = int(position[0] * 1e6 * sp)
                    startrobot.y = int(position[1] * 1e6 * sp)
                    startrobot.z = int(position[2] * 1e6 * sp)
                    startrobot.a = int(euler[0] * 1e6 * (180 / np.pi))
                    startrobot.b = int(euler[1] * 1e6 * (180 / np.pi))
                    startrobot.c = int(euler[2] * 1e6 * (180 / np.pi))

                # ToRobot構造体に代入
                pos.x = position[0] * sp
                pos.y = position[1] * sp
                pos.z = position[2] * sp
                pos.a = euler[0] * (180 / np.pi)
                pos.b = euler[1] * (180 / np.pi)
                pos.c = euler[2] * (180 / np.pi)

                time_current = time.time()
                sample_sec = time_current - time_prev
                time_prev = time_current

                print(f"pos.x:{np.round(pos.x, 3)}")
                print(f"pos.y:{np.round(pos.y, 3)}")
                print(f"pos.z:{np.round(pos.z, 3)}")
                print(f"pos.a:{np.round(pos.a, 3)}")
                print(f"pos.b:{np.round(pos.b, 3)}")
                print(f"pos.c:{np.round(pos.c, 3)}")

                rel.x = int((pos.x - startpos.x) * 1e6)
                rel.y = int((pos.y - startpos.y) * 1e6)
                rel.z = int((pos.z - startpos.z) * 1e6)
                rel.a = int((pos.a - startpos.a) * 1e6)
                rel.b = int((pos.b - startpos.b) * 1e6)
                rel.c = int((pos.c - startpos.c) * 1e6)

                send.pos.x = startrobot.x + rel.x
                send.pos.y = startrobot.y + rel.y
                send.pos.z = startrobot.z + rel.z
                send.pos.a = startrobot.a + rel.a
                send.pos.b = startrobot.b + rel.b
                send.pos.c = startrobot.c + rel.c
                send.cmnd = 2
                send.pad_ += 1

                txbuf = send.OnSendData()
                # print(f"Packed data:{txbuf}")
                # print(len(txbuf))

                # received = ToRobot.from_bytes(txbuf)
                # print(f"Unpacked ToRobot:pos:{received}")

                # サーバーへ送信
                csocket.SendTo(txbuf)

                # now_time = time.time()
                # print(f"time:{now_time-start_time}")

                theata_j2_grav = (np.pi / 2) - theata_rad[1]
                theata_j3_grav = theata_rad[2] - theata_rad[1]
                # print(f"theata_j2_grav:{theata_j2_grav}, theata_j3_grav:{theata_j3_grav}")

                # 重力補償計算
                tq_grav_j2 = -weight_j36 * (
                    tp_grav_j36 * (length_36 * np.cos(theata_j3_grav))
                    + tp_grav_j23 * (length_23[2][0] * np.cos(theata_j2_grav))
                ) + -weight_j23 * tp_grav_j23 * length_23[2][0] * np.cos(theata_j2_grav)
                tq_grav_j3 = (
                    tp_grav_j3 * weight_j36 * length_36 * np.cos(theata_j3_grav)
                )
                print(f"tq_j2_grav:{tq_grav_j2}, tq_j3_grav:{tq_grav_j3}")

                tq_gravcom = np.array([0, tq_grav_j2, tq_grav_j3, 0, 0, 0]).astype(
                    np.int16
                )

                tq_output_gravcom = tq_output - tq_gravcom
                print(f"gravcomoutput{tq_output_gravcom}")

                # ダイレクト操作の処理
                if (all(getdata_result_pos) == True) and (
                    all(getdata_result_tq) == True
                ):
                    pos_diff = np.where(
                        np.abs(tq_output_gravcom) > tq_threshold,
                        (tq_threshold - tq_output_gravcom) * tp,
                        0,
                    )
                    pos_ref += pos_diff * kr
                    pos_err = pos_ref - pos_output
                    pos_input = pos_ref - pos_err * kp

                # デバック用
                for i in range(len(DXL_IDs)):
                    print(
                        f"ID:{DXL_IDs[i]}, diff:{np.round(pos_diff[i], 2)}, ref:{np.round(pos_ref[i], 2)}, err:{np.round(pos_err[i], 2)}, input:{np.round(pos_input[i], 2)}, J{[i+1]}:{np.round(theata_deg[i], 2)}° "
                    )

                # 例外処理
                exceptprocess.allexceptprocess(
                    pos_err, pos_output, pos_init_goal, dxl.getdata_result
                )

                # 動作命令
                if (all(getdata_result_pos) == True) and (
                    all(getdata_result_tq) == True
                ):
                    dxl.writeGoalPosition(DXL_IDs, pos_input)
                else:
                    print(f"no Write to Dynamixel")

            print(f"com_err_count:{exceptprocess.com_err_count}")
            print(f"getdata_result:{dxl.getdata_result}")
            print(f"getdata_result_pos:{getdata_result_pos}")
            print(f"getdata_result_tq:{getdata_result_tq}")

            now_time = time.time()

            if (now_time - start_time) <= 0.009:
                time.sleep(0.010 - (now_time - start_time))
            cycle_time = time.time() - start_time
            print(f"processtime:{now_time - start_time}")
            print(f"cycletime:{cycle_time}")

    except KeyboardInterrupt:
        print("-----------| while stop |-----------")
    except ValueError as err:
        print(err)

    finally:
        dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
        if dxl.getdata_result != False:
            initializeend.endprocess(pos_init_goal, pos_output)
            dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
        print("-----------| end code |-----------")


if __name__ == "__main__":
    __init__()
