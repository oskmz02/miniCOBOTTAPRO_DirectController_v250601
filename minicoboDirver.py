from initialize_end import InitializeEnd
from except_process import ExceptProcess
from minicoboGravityCompensation import MiniCoboGravCom
from minicoboPose import CoboKinematic, MiniCoboPose, ToRobot, Axis6i
from minicoboClient import CSocket, QSocket

import queue
import time
from Dynamixel import Dynamixel, Test_Dynamixel
import numpy as np
import copy as cp


class MiniCoboThread:
    def __init__(
        self,
        st_minicobo_q: queue.Queue,
        cmd_minicobo_q: queue.Queue,
        dxl: Dynamixel,
        DXL_IDs,
        ctrlmode,
    ):
        self.st_minicobo_init = False
        self.st_com = False

        self.m_st_minicobo_q = st_minicobo_q
        self.m_cmd_minicobo_q = cmd_minicobo_q

    def startUpSequence(self, dxl: Dynamixel, DXL_IDs, ctrlmode):
        tst = Test_Dynamixel()
        dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
        dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
        dxl.writeOperatingMode(
            DXL_IDs, ctrlmode * len(DXL_IDs)
        )  # モード5：位置電流制御モード
        dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
        tst.testReadSettings(dxl, DXL_IDs)
        time.sleep(2)
        print(f"data clearning...")

        dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
        tst.testReadSettings(dxl, DXL_IDs)
        init_getdata_result = dxl.getdata_result_array
        tst_time = tst.elapsed_time * 1000  # [msec]
        if 9.1 > tst_time or tst_time > 9.9:
            st_comtime = False
        else:
            st_comtime = True

        print(f"communication result:{dxl.getdata_result_array}")

        if all(init_getdata_result) != True:
            dxl.setRecommendedValue(DXL_IDs)
            tst.testReadSettings(dxl, DXL_IDs)
            time.sleep(2)
        elif (all(init_getdata_result) == True) and (st_comtime != True):
            dxl.setRecommendedValue(DXL_IDs)
            tst.testReadSettings(dxl, DXL_IDs)
            time.sleep(2)
        else:
            self.st_com = True
            self.st_minicobo_init = True
            dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))
            print(f"[MINICOBO] Set up successfull")

    def directControlPos(self, dxl: Dynamixel, DXL_IDs):
        dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))
        self.m_st_minicobo_q.put("AUTO")

        tq_threshold = np.array([6, 11, 10, 12, 8, 6])  # 位置用
        pos_err = np.zeros(len(DXL_IDs))
        pos_input = np.zeros(len(DXL_IDs))

        tq_gravcom = np.zeros(len(DXL_IDs))
        tq_diff = np.zeros(len(DXL_IDs))
        tq_diff_prev = np.zeros(len(DXL_IDs))

        del_t = np.array([0.010] * len(DXL_IDs))
        alpha = np.array([144.5, 148.0, 50.2, 216.0, 422.0, 422.0])
        beta = np.array([1.1, 1.2, 4.0, 8.2, 23.0, 22.5])
        gamma_0 = np.array([20.8, 19.8, 7.8, 17.4, 6.2, 3.2])
        gamma_1 = np.array([18.0, 17.0, 10.0, 14.0, 10.0, 10.0])
        gamma_v = np.zeros(len(DXL_IDs))
        v_0 = np.array([168.0, 168.0, 96.0, 216.0, 62.0, 96.0])

        a_cul = np.zeros(len(DXL_IDs))
        v_diff = np.zeros(len(DXL_IDs))
        v_ref = np.zeros(len(DXL_IDs))
        a_lim = np.array([2835.0, 2835.0, 1655.0, 4136.0, 4136.0, 4136.0])
        v_lim = np.array([635.0, 635.0, 375.0, 635.0, 635.0, 870.0])

        # クラス定義
        initializeend = InitializeEnd(dxl, DXL_IDs)
        exceptprocess = ExceptProcess(dxl, DXL_IDs, self.st_com)
        grvcom = MiniCoboGravCom(dxl, DXL_IDs)
        poseinit = MiniCoboPose(dxl, DXL_IDs)
        mcpose = MiniCoboPose(dxl, DXL_IDs)  # miniCOBOTTAPROの姿勢
        pose = MiniCoboPose(dxl, DXL_IDs)  # 内部目標姿勢
        poseprop = MiniCoboPose(dxl, DXL_IDs)  # 内部目標姿勢候補
        poseprev = MiniCoboPose(dxl, DXL_IDs)  # 前ループ目標姿勢
        csocket = CSocket()
        qsocket = QSocket()

        pad_ = 0
        com_statu_prev = True

        # kinematic.inv()
        # 位置初期化
        pos_init_goal = initializeend.initializepostion()

        PROFILEACCELERATION = np.array([256, 256, 256, 256, 256, 256])
        PROFILEVELOCITY = np.array([128, 128, 128, 128, 128, 128])
        dxl.writeProfileAcceleration(DXL_IDs, PROFILEACCELERATION)
        dxl.writeProfileVelocity(DXL_IDs, PROFILEVELOCITY)
        POSDGAIN = np.array([600, 720, 600, 600, 400, 400])
        POSIGAIN = np.array([48, 18, 6, 64, 60, 20])
        POSPGAIN = np.array([500, 600, 600, 480, 320, 320])
        dxl.writePositionDGain(DXL_IDs, POSDGAIN)
        dxl.writePositionIGain(DXL_IDs, POSIGAIN)
        dxl.writePositionPGain(DXL_IDs, POSPGAIN)

        # 角度とトルクの初期化
        pos_ref = np.array(dxl.readPresentPosition(DXL_IDs)).astype(np.float64)
        pos_ref_prop = cp.copy(pos_ref)
        pos_output = np.array(dxl.readPresentPosition(DXL_IDs))
        tq_output = np.array(dxl.readPresentCurrent(DXL_IDs))
        time.sleep(0.006)

        try:
            print("--------------| while start |--------------")
            timer01 = time.time()

            while True:
                t_st = time.time()

                try:
                    m_cmd = self.m_cmd_minicobo_q.get_nowait()
                except queue.Empty:
                    m_cmd = None

                if m_cmd == "STOP":
                    break

                t_600m = time.time() * 1000

                # 受信
                pos_output_rx = np.array(dxl.readPresentPosition(DXL_IDs))
                getdata_result_pos = np.array(dxl.getdata_result_array)

                t_600p = time.time() * 1000

                tq_output_rx = np.array(dxl.readPresentCurrent(DXL_IDs))
                getdata_result_tq = np.array(dxl.getdata_result_array)

                t_600t = time.time() * 1000

                # 毎ループの処理部分
                if (all(getdata_result_pos) != True) or (
                    all(getdata_result_tq) != True
                ):
                    # 通信失敗 → カウンタを初期化
                    exceptprocess.recovery_wait_counter = 2
                    exceptprocess.comErr(False)  # 失敗したらなのでFalse直入れ
                    com_statu_prev = False
                    self.m_st_minicobo_q.put("ERROR")

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
                        # print("通信復旧後スキップ")
                        self.m_st_minicobo_q.put("ERROR")
                    else:
                        com_statu_prev = True
                        # カウントダウン終了、正常処理開始
                        pos_output = pos_output_rx
                        tq_output = tq_output_rx
                    # ここに演算処理を書く

                else:
                    com_statu_prev = True
                    poseprev.setPose(pos_ref - pos_init_goal, 1)
                    if pose.send.pad_ == 0:
                        time_prev = time.time()
                    # 通信成功の場合
                    # 受信データの代入
                    pos_output = pos_output_rx
                    tq_output = tq_output_rx
                    print(f"tq:{tq_output}, pos_output:{pos_output}")

                    # 角度の算出
                    theata = pos_output - pos_init_goal
                    mcpose.setPose(theata, 1)

                    t_601 = time.time() * 1000

                    # 重力補償
                    tq_gravcom = grvcom.gravComSimple(mcpose.rad)
                    tq_output_gravcom = tq_output - tq_gravcom
                    print(f"gravcomoutput{tq_output_gravcom}")

                    t_602 = time.time() * 1000

                    # ダイレクト操作の処理
                    if (all(getdata_result_pos) == True) and (
                        all(getdata_result_tq) == True
                    ):
                        # 制御ブロック
                        tq_diff = np.where(
                            np.abs(tq_output_gravcom) > tq_threshold,
                            tq_threshold - tq_output_gravcom,
                            0,
                        )
                        tq_diff_dot = (tq_diff - tq_diff_prev) / del_t
                        tq_diff_prev = tq_diff

                        gamma_v = gamma_0 + gamma_1 * np.exp(-(abs(v_ref) / v_0))

                        a_cul = alpha * tq_diff + beta * tq_diff_dot - gamma_v * v_ref
                        a_ref = np.where(
                            np.abs(a_lim) < np.abs(a_cul),
                            np.sign(a_cul) * a_lim,
                            a_cul,
                        )

                        v_diff_prop = v_diff + a_ref * del_t

                        # 予測速度が限界を超える場合は加速無し
                        a_ref = np.where(np.abs(v_diff_prop) > v_lim, 0.0, a_ref)

                        v_diff += a_ref * del_t
                        print(f"J3 v_diff : {np.round(v_diff[2], 2)}")
                        v_ref = np.where(
                            np.abs(v_lim) <= np.abs(v_diff),
                            np.sign(v_diff) * v_lim,
                            v_diff,
                        )

                        pos_diff_prop = v_ref * del_t
                        pos_ref_prop += pos_diff_prop

                        poseprop.setPose(pos_ref_prop - pos_init_goal, 1)
                        """ここで速度超過の判定"""
                        del_pose_prop = [
                            poseprop.pos.x - poseprev.pos.x,
                            poseprop.pos.y - poseprev.pos.y,
                            poseprop.pos.z - poseprev.pos.z,
                        ]
                        print(
                            f"del_x:{np.round(del_pose_prop[0]/del_t[0], 2)}, del_y:{np.round(del_pose_prop[1]/del_t[0], 2)}, del_z:{np.round(del_pose_prop[2]/del_t[0], 2)}"
                        )
                        v_prop_norm = np.linalg.norm(del_pose_prop) / del_t[0]
                        print(f"v_prop_norm : {np.round(v_prop_norm, 2)}")
                        if v_prop_norm > 225:
                            kr = np.array([225 / v_prop_norm] * len(DXL_IDs))
                            v_ref *= kr
                            v_diff *= kr
                        else:
                            kr = np.array([1.0] * len(DXL_IDs))

                        pos_diff = pos_diff_prop * kr
                        pos_ref_prop = pos_ref_prop - pos_diff_prop + pos_diff
                        pos_ref += pos_diff

                        t_603 = time.time() * 1000

                        pose.setPose(pos_ref - pos_init_goal, 2)
                        del_pose = [
                            pose.pos.x - poseprev.pos.x,
                            pose.pos.y - poseprev.pos.y,
                            pose.pos.z - poseprev.pos.z,
                        ]

                        v_norm = np.linalg.norm(del_pose) / del_t[0]
                        print(f"v_norm : {v_norm}")

                        pos_err = pos_ref - pos_output
                        pos_input = pos_ref

                        # print(f"Ts_adj : {pose.Ts_adj}")
                        toquest = pose.toquest.OnSendData()
                        # print(f"toquest_byte : {len(toquest)}")
                        qsocket.SendToQuest(toquest)

                        torobot = pose.torobot.OnSendData()
                        csocket.SendTo(torobot)

                    # デバック用
                    for i in range(len(DXL_IDs)):
                        print(
                            f"ID:{DXL_IDs[i]}, diff:{np.round(pos_diff[i], 2)}, ref:{np.round(pos_ref[i], 2)}, err:{np.round(pos_err[i], 2)}, input:{np.round(pos_input[i], 2)}, J{[i+1]}:{np.round(mcpose.deg[i], 2)}° "
                        )

                    # 例外処理
                    exceptprocess.allexceptprocess(
                        pos_err, pos_output, pos_init_goal, dxl.getdata_result
                    )

                    # 動作命令
                    dxl.writeGoalPosition(DXL_IDs, pos_input)
                    t_604 = time.time() * 1000
                    self.m_st_minicobo_q.put("NORMAL")
                    for i in range(len(DXL_IDs)):
                        print(f"a_ref:{np.round(a_ref[i] * kr[i], 2)}")
                    for i in range(len(DXL_IDs)):
                        print(f"v_ref:{np.round(v_ref[i] * kr[i], 2)}")
                print(f"com_err_count:{exceptprocess.com_err_count}")
                print(f"getdata_result:{dxl.getdata_result}")
                print(f"getdata_result_pos:{getdata_result_pos}")
                print(f"getdata_result_tq:{getdata_result_tq}")

                t_605 = time.time() * 1000
                t_now = time.time()
                print(
                    f"t_p : {np.round(t_600p - t_600m, 3)}, {np.round(t_600t - t_600p, 3)}, {np.round(t_601 - t_600t, 3)}, {np.round(t_602 - t_601, 3)}, {np.round(t_603 - t_602, 3)}, {np.round(t_604 - t_603, 3)}, {np.round(t_605 - t_604, 3)}"
                )
                print(f"processtime:{t_now - t_st}")
                if (t_now - t_st) <= 0.009:
                    time.sleep(0.010 - (t_now - t_st))
                cycle_time = time.time() - t_st
                del_t = np.array([cycle_time] * len(DXL_IDs))

                # print(f"cycletime:{cycle_time}")

        except KeyboardInterrupt:
            print("-----------| KeyboardInterrupt |-----------")
        except ValueError as err:
            self.m_st_minicobo_q.put("ERROR")
            print(err)

        finally:
            dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
            self.m_st_minicobo_q.put("AUTO")
            if dxl.getdata_result != False:
                initializeend.endprocess(pos_init_goal, pos_output)
                dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
                self.m_st_minicobo_q.put("IDLE")
            else:
                self.m_st_minicobo_q.put("EMC")
            # print("-----------| end code |-----------")

    def directControlCurPos(self, dxl: Dynamixel, DXL_IDs):
        dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))
        self.m_st_minicobo_q.put("AUTO")

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
            [
                length_01,
                length_12,
                length_23,
                length_34,
                length_45,
                length_56,
                length_6end,
            ]
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
        exceptprocess = ExceptProcess(dxl, DXL_IDs, self.st_com)
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
        pos_init_goal = initializeend.initializepostion()

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
        pos_ref = np.array(dxl.readPresentPosition(DXL_IDs)).astype(np.float64)
        pos_output = np.array(dxl.readPresentPosition(DXL_IDs))
        tq_output = np.array(dxl.readPresentCurrent(DXL_IDs))
        time.sleep(0.009)

        try:
            print("--------------| while start |--------------")
            timer01 = time.time()

            while True:
                t_st = time.time()
                if send.pad_ == 0:
                    time_prev = time.time()

                try:
                    m_cmd = self.m_cmd_minicobo_q.get_nowait()
                except queue.Empty:
                    m_cmd = None

                if m_cmd == "STOP":
                    break

                # 受信
                pos_output_rx = np.array(dxl.readPresentPosition(DXL_IDs))
                getdata_result_pos = np.array(dxl.getdata_result_array)

                tq_output_rx = np.array(dxl.readPresentCurrent(DXL_IDs))
                getdata_result_tq = np.array(dxl.getdata_result_array)

                # 毎ループの処理部分
                if (all(getdata_result_pos) != True) or (
                    all(getdata_result_tq) != True
                ):
                    # 通信失敗 → カウンタを初期化
                    exceptprocess.recovery_wait_counter = 3
                    exceptprocess.comErr(False)  # 失敗したらなのでFalse直入れ
                    com_statu_prev = False
                    self.m_st_minicobo_q.put("ERROR")

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
                        # print("通信復旧後スキップ")
                        self.m_st_minicobo_q.put("ERROR")
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
                    ) + -weight_j23 * tp_grav_j23 * length_23[2][0] * np.cos(
                        theata_j2_grav
                    )
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
                    dxl.writeGoalPosition(DXL_IDs, pos_input)
                    self.m_st_minicobo_q.put("NORMAL")

                print(f"com_err_count:{exceptprocess.com_err_count}")
                print(f"getdata_result:{dxl.getdata_result}")
                print(f"getdata_result_pos:{getdata_result_pos}")
                print(f"getdata_result_tq:{getdata_result_tq}")

                t_now = time.time()

                if (t_now - t_st) <= 0.009:
                    time.sleep(0.010 - (t_now - t_st))
                cycle_time = time.time() - t_st
                print(f"processtime:{t_now - t_st}")
                print(f"cycletime:{cycle_time}")

        except KeyboardInterrupt:
            print("-----------| KeyboardInterrupt |-----------")
        except ValueError as err:
            self.m_st_minicobo_q.put("ERROR")
            print(err)

        finally:
            dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on
            self.m_st_minicobo_q.put("AUTO")
            if dxl.getdata_result != False:
                initializeend.endprocess(pos_init_goal, pos_output)
                dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
                self.m_st_minicobo_q.put("IDLE")
            else:
                self.m_st_minicobo_q.put("ERROR")
            # print("-----------| end code |-----------")
