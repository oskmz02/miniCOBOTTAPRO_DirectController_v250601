import os

# from Dynamixel import Dynamixel


class ExceptProcess:
    """例外処理クラス
    DXL_IDs: Dynamixel ID (list)
    """

    def __init__(self, dxl, DXL_IDs, st_com):
        self.dxl = dxl
        self.DXL_IDs = DXL_IDs
        self.poserrthreshold = 600  # pos_refとpos_inputの差の閾値
        self.com_err_count = 0
        self.recovery_wait_counter = 0  # 通信復帰時の待ちカウンター
        self.com_statu = st_com
        self.com_statu_prev = st_com

    def allexceptprocess(self, pos_err, pos_output, init_goal_pos, getdata_result):
        self.com_statu_flag = self.comErrCount(getdata_result, self.com_statu)
        self.abnomalStop(self.com_statu_flag)
        self.com_statu_prev = self.com_statu
        for i in range(len(self.DXL_IDs)):
            self.bigpos_err(i, pos_err)
            self.bigoutput_pos(i, pos_output, init_goal_pos)

    def comErr(self, getdata_result):
        self.com_statu_flag = self.comErrCount(getdata_result, self.com_statu)
        self.abnomalStop(self.com_statu_flag)
        self.com_statu_prev = self.com_statu

    # 例外処理１：pos_errが大きい（暴走防止）
    def bigpos_err(self, i, pos_err):
        if pos_err[i] >= self.poserrthreshold:
            raise ValueError("ーーERROR :  過負荷を検知しました）ーー")

    # 例外処理２：pos_outputが大きい（配線保護）
    def bigoutput_pos(self, i, pos_output, init_goal_pos):
        if (init_goal_pos[i] - 4096) >= pos_output[i] or pos_output[i] >= (
            init_goal_pos[i] + 4096
        ):  # 初期値から一回転
            if i != 5:  # 6番目だけ例外
                raise ValueError("ーーERROR :  pos_outputが大きい（配線保護）ーー")

    def comErrCount(self, getdata_result, com_statu):
        if getdata_result == False:
            self.com_err_count += 1
            print(f"com_err_count:{self.com_err_count}")
        # else:
        # self.com_err_count = 0  # 成功したらリセット
        if self.com_err_count >= 30:
            com_statu = False

        return com_statu

    # 緊急強制終了：モータ通信不良を検知
    def abnomalStop(self, com_statu):
        if com_statu == False:
            self.dxl.writeTorqueEnable(self.DXL_IDs, [0] * len(self.DXL_IDs))
            self.dxl.writeTorqueEnable(self.DXL_IDs, [0, 1, 0, 0, 0, 0])
            print(f"通信不良を検知しました．強制終了します．")
            print(f"J2のみ固定しています．再起動時はアームを支えてください．")
            print(f"mini COBOTTA PROを再起動してください")
            raise ValueError("Emergency STOP")
            # os._exit(1)
