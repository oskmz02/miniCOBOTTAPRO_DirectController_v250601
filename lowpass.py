import numpy as np


class LowPass:
    def __init__(self, cd1, cd2, cd3, cd4):
        self.m_vel = self.m_velmax = 0.0
        self.m_acc = self.m_accmax = 0.0

        self.init = 0.0
        self.vel_lim = 0.0
        self.acc_lim = 0.0
        self.pos_last = 0.0
        self.vel_last = 0.0

        self.x1 = self.x2 = self.x3 = self.x4 = 0.0
        self.Ad = np.zeros((4, 4))
        self.Bd = np.zeros(4)
        self.Cd = np.array([cd1, cd2, cd3, cd4])

    def initialize(self, cutoff_hz, sample_sec, data, vel_lim=0.0, acc_lim=0.0):
        omega = cutoff_hz * 2 * np.pi
        T = 1 / omega
        ratio = sample_sec / (T + sample_sec)

        self.Ad[0][0] = self.Ad[1][1] = self.Ad[2][2] = self.Ad[3][3] = 1 - ratio
        self.Ad[1][0] = self.Ad[2][1] = self.Ad[3][2] = ratio

        self.Bd[0] = ratio

        self.init = data
        self.vel_lim = vel_lim
        self.acc_lim = acc_lim
        self.pos_last = 0.0
        self.vel_last = 0.0
        self.x1 = self.x2 = self.x3 = self.x4 = 0.0
        self.m_vel = self.m_velmax = 0.0
        self.m_acc = self.m_accmax = 0.0

    def data(self, input_value):
        pos = input_value - self.init
        vel = pos - self.pos_last
        acc = vel - self.vel_last

        self.m_vel = abs(vel)
        self.m_acc = abs(acc)

        if self.m_velmax < self.m_vel:
            self.m_velmax = self.m_vel
        if self.m_accmax < self.m_acc:
            self.m_accmax = self.m_acc

        if self.acc_lim != 0.0:
            acc = max(-self.acc_lim, min(self.acc_lim, acc))

        vel = self.vel_last + acc
        if self.vel_lim != 0.0:
            vel = max(-self.vel_lim, min(self.vel_lim, vel))

        pos = self.pos_last + vel

        x1_tmp = (
            self.Ad[0][0] * self.x1
            + self.Ad[0][1] * self.x2
            + self.Ad[0][2] * self.x3
            + self.Ad[0][3] * self.x4
            + self.Bd[0] * pos
        )
        x2_tmp = (
            self.Ad[1][0] * self.x1
            + self.Ad[1][1] * self.x2
            + self.Ad[1][2] * self.x3
            + self.Ad[1][3] * self.x4
            + self.Bd[1] * pos
        )
        x3_tmp = (
            self.Ad[2][0] * self.x1
            + self.Ad[2][1] * self.x2
            + self.Ad[2][2] * self.x3
            + self.Ad[2][3] * self.x4
            + self.Bd[2] * pos
        )
        x4_tmp = (
            self.Ad[3][0] * self.x1
            + self.Ad[3][1] * self.x2
            + self.Ad[3][2] * self.x3
            + self.Ad[3][3] * self.x4
            + self.Bd[3] * pos
        )

        pos = (
            self.Cd[0] * self.x1
            + self.Cd[1] * self.x2
            + self.Cd[2] * self.x3
            + self.Cd[3] * self.x4
        )

        self.x1, self.x2, self.x3, self.x4 = x1_tmp, x2_tmp, x3_tmp, x4_tmp
        self.pos_last = pos
        self.vel_last = vel

        return pos + self.init
