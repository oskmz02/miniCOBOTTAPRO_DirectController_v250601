import time
from initialize_end import InitializeEnd
import collections
from Dynamixel import Dynamixel, Test_Dynamixel
import os
import numpy as np


def __init__():
    DXL_IDs = [1, 2]  # 動かしたいサーボのIDを指定
    BAUDRATE = 4000000  # 基本変更しなくてよし
    DEVICENAME = "/dev/ttyUSB0"  # 接続されているポート指定
    dxl = Dynamixel(DEVICENAME, BAUDRATE)  # Dynamixelクラスイニシャライズ
    dxl.setRecommendedValue(DXL_IDs)  # 動かすサーボの初期化
    time.sleep(1)

    dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
    time.sleep(1)
    dxl.writeOperatingMode(DXL_IDs, [3] * len(DXL_IDs))  # モード1：速度制御モード
    time.sleep(1)
    dxl.writeTorqueEnable(DXL_IDs, [1] * len(DXL_IDs))  # Torque on

    tst = Test_Dynamixel()
    tst.testReadSettings(dxl, DXL_IDs)

    print("------| initialize Current based Posision Mode |------")
    presentPosD = dxl.readPresentPositionDGain(DXL_IDs)
    print(f"presentPosD:{presentPosD}")
    dxl.writePositionDGain(DXL_IDs, [1200] * len(DXL_IDs))
    dxl.writePositionIGain(DXL_IDs, [500] * len(DXL_IDs))
    dxl.writePositionPGain(DXL_IDs, [800] * len(DXL_IDs))

    time.sleep(1)

    currentBasedPositionDirectControl(dxl, DXL_IDs)  # 位置電流制御モード


def currentBasedPositionDirectControl(dxl: Dynamixel, DXL_IDs):
    tq_threshold = np.array([10] * len(DXL_IDs))
    # tp = 0.2
    # kp = 0.3
    # kr = 0.5

    pos_ref = np.array(dxl.readPresentPosition(DXL_IDs)).astype(np.float64)

    pos_err = np.zeros(len(DXL_IDs))
    pos_input = np.zeros(len(DXL_IDs))
    pos_diff = np.zeros(len(DXL_IDs))

    tq_ref = np.array([60] * len(DXL_IDs))
    tq_input = np.zeros(len(DXL_IDs))

    tq_gravcom = np.zeros(len(DXL_IDs))
    tq_diff = np.zeros(len(DXL_IDs))
    tq_diff_prev = np.zeros(len(DXL_IDs))
    # tq_diff_dot_prev = np.zeros(len(DXL_IDs))
    del_t = np.array([0.010] * len(DXL_IDs))
    alpha = np.array([20] * len(DXL_IDs))
    beta = np.array([4] * len(DXL_IDs))
    gamma_0 = np.array([2.0] * len(DXL_IDs))
    gamma_1 = np.array([4.0] * len(DXL_IDs))
    gamma_v = np.zeros(len(DXL_IDs))
    a_cul = np.zeros(len(DXL_IDs))
    # a_thr = np.array([0.1] * len(DXL_IDs))
    # v_diff_1 = np.zeros(len(DXL_IDs))
    v_diff = np.zeros(len(DXL_IDs))
    v_ref = np.zeros(len(DXL_IDs))
    v_abs = np.zeros(len(DXL_IDs))
    v_0 = np.array([3.0] * len(DXL_IDs))
    # a_lim_rev = np.array([4040])
    a_lim = np.array([17060] * len(DXL_IDs))
    v_lim = np.array([853] * len(DXL_IDs))

    initializeend = InitializeEnd(dxl, DXL_IDs)
    pos_init_goal = initializeend.initializepostion(pos_ref)

    dxl.writeProfileAcceleration(DXL_IDs, [196] * len(DXL_IDs))
    dxl.writeProfileVelocity(DXL_IDs, [82] * len(DXL_IDs))

    dxl.writePositionDGain(DXL_IDs, [600] * len(DXL_IDs))
    dxl.writePositionIGain(DXL_IDs, [100] * len(DXL_IDs))
    dxl.writePositionPGain(DXL_IDs, [400] * len(DXL_IDs))

    pos_output = np.array(dxl.readPresentPosition(DXL_IDs))
    tq_output = np.array(dxl.readPresentCurrent(DXL_IDs))

    pos_err_prev = pos_init_goal - pos_output

    pos_output_prev = pos_output
    pos_output_prev02 = pos_output

    print("--------------| while start |--------------")
    try:
        while True:
            start_time = time.time()
            pos_output = np.array(dxl.readPresentPosition(DXL_IDs))
            tq_output = np.array(dxl.readPresentCurrent(DXL_IDs))

            # os.system("clear")
            print(f"tq:{tq_output}, pos_output:{pos_output}")

            theata = pos_output - pos_init_goal
            theata_rad = np.round((theata * (np.pi / 2048)), 12)

            tq_output_gravcom = tq_output - tq_gravcom  # + np.abs(pos_err_prev)

            # 制御ブロック(試験運用版)
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

            v_diff += a_ref * del_t
            v_ref = np.where(
                np.abs(v_lim) < np.abs(v_diff),
                np.sign(v_diff) * v_lim,
                v_diff,
            )

            pos_diff = v_ref * del_t
            pos_ref += pos_diff
            pos_err = pos_ref - pos_output
            pos_input = pos_ref

            pos_err_prev = pos_err
            tq_input = tq_ref + tq_output_gravcom

            # デバック用
            for i in range(len(DXL_IDs)):
                print(
                    f"ID: {DXL_IDs[i]}\n"
                    f"  diff   : {np.round(pos_diff[i], 2)}\n"
                    f"  ref    : {np.round(pos_ref[i], 2)}\n"
                    f"  err    : {np.round(pos_err[i], 2)}\n"
                    f"  tq_diff: {np.round(tq_diff[i], 2)}\n"
                    f"  input  : {np.round(pos_input[i], 2)}\n"
                    f"  output : {np.round(pos_output[i], 2)}\n"
                    f"  tq_og : {np.round(tq_output_gravcom[i], 2)}\n"
                    f"  tq_input : {np.round(tq_input[i], 2)}\n"
                    f"  tq_diff : {np.round(tq_diff[i], 2)}\n"
                    f"  a_ref : {np.round(a_ref[i], 2)}\n"
                    f"  v_ref : {np.round(v_ref[i], 2)}\n"
                )

            # dxl.writeGoalCurrent(DXL_IDs, tq_input)
            dxl.writeGoalPosition(DXL_IDs, pos_input)

            now_time = time.time()
            if (now_time - start_time) <= 0.009:
                time.sleep(0.010 - (now_time - start_time))
            now_time = time.time()
            del_t = np.array([now_time - start_time] * len(DXL_IDs))
            print(f"cycletime:{now_time - start_time}")

    except KeyboardInterrupt:
        dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
        print("-----------| while stop |-----------")
    finally:
        dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off
        print("-----------| end code |-----------")


def torqueAndVelocityControl(dxl: Dynamixel, DXL_IDs):

    # 設定：トルク閾値、追従動作量、PIDゲイン
    torque_threshold = 35  # トルク閾値（適宜調整）
    P_GAIN = 32  # Pゲイン（追従速度の調整に影響）
    I_GAIN = 0.05  # Iゲイン（位置の安定化に影響）
    D_GAIN = 0.01  # Dゲイン（応答の滑らかさに影響）
    adjust_amount = 4  # 位置調整の量 XM430
    # adjust_amount = 32      # 位置調整の量 XC330
    adjustment = 0
    error = 0
    integral_errors = 0
    derivative = 0
    previous_errors = 0
    smoothed_vel = 0
    attnCoef = 0.2
    attnCoef_f = 0.1
    time.sleep(1)

    # 追従位置のフィルタリング用の変数
    previous_errors = [0] * len(DXL_IDs)
    integral_errors = [0] * len(DXL_IDs)

    # new_positionの移動平均フィルタ用のキューと設定
    MOVING_AVERAGE_WINDOW = 8  # 移動平均のウィンドウサイズ XM430:5~10 XC330:16~
    position_history = [collections.deque([0] * MOVING_AVERAGE_WINDOW) for _ in DXL_IDs]

    TORQUE_AVERAGE_WINDOW = 1  # トルク平均サイズ
    torque_history = [collections.deque([0] * TORQUE_AVERAGE_WINDOW) for _ in DXL_IDs]

    print("--------------| while start |--------------")

    try:
        while True:
            current_vel = dxl.readPresentVelocity(DXL_IDs)  # 現在の速度を取得
            current_torques = dxl.readPresentCurrent(DXL_IDs)  # 現在のトルクを取得

            for i, torque in enumerate(current_torques):
                """
                torque_history[i].append(torque0)
                torque_history[i].popleft()
                torque= sum(torque_history[i]) / TORQUE_AVERAGE_WINDOW
                """
                # torque = torque0
                # Calculate direction and error based on detected torque
                if abs(torque) > torque_threshold:  # If external torque detected
                    direction = -1 if torque > 0 else 1

                    error = adjust_amount * direction  # Set error based on torque
                else:
                    # error = error * attnCoef  # Set error to zero when no external force is detected
                    error = 0

                # PID calculation with decay effect when no torque is present
                integral_errors[i] += error
                derivative = error - previous_errors[i]
                previous_errors[i] = error

                # Compute adjustment with PID
                adjustment = (
                    P_GAIN * error + I_GAIN * integral_errors[i] + D_GAIN * derivative
                )

                # Filtered velocity output
                new_velocity = current_vel[i] + adjustment
                position_history[i].append(new_velocity)
                position_history[i].popleft()
                smoothed_vel = sum(position_history[i]) / MOVING_AVERAGE_WINDOW

                # Apply smoothed velocity as goal
                # if文でsmooth_velとトルクの方向が逆方向だったら即座に減速する
                if (torque < 0 and smoothed_vel < 0) or (
                    torque > 0 and smoothed_vel > 0
                ):
                    smoothed_vel = smoothed_vel * attnCoef_f
                    print("oposit direction")
                    dxl.writeGoalVelocity([DXL_IDs[i]], [smoothed_vel])
                else:
                    dxl.writeGoalVelocity([DXL_IDs[i]], [smoothed_vel])

                # Decay effect for smoothing if no external torque
                if abs(torque) < torque_threshold:
                    smoothed_vel *= attnCoef
                    integral_errors[i] *= attnCoef
                    derivative *= attnCoef
                    dxl.writeGoalVelocity([DXL_IDs[i]], [smoothed_vel])

                # XM430-W210-R
                if abs(smoothed_vel) < 0.1:
                    smoothed_vel = 0
                    integral_errors[i] = 0
                    derivative = 0
                if abs(error) < 0.1:
                    error = 0
                """
                #XC330-W288-T
                if abs(smoothed_vel) < 1:
                    smoothed_vel = 0
                    integral_errors[i] = 0
                    derivative = 0
                if abs(error) < 0.1:
                    error = 0
                """

            print(
                "Torque:",
                torque,
                "Smoothed Vel:",
                smoothed_vel,
                "Position:",
                dxl.readPresentPosition(DXL_IDs),
            )

            # time.sleep(0.01)  # 追従の頻度（適宜調整）

    except KeyboardInterrupt:
        print("-----------| end code |-----------")
        dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off

    finally:
        print("-----------| end code |-----------")
        dxl.writeTorqueEnable(DXL_IDs, [0] * len(DXL_IDs))  # Torque off


if __name__ == "__main__":
    __init__()
