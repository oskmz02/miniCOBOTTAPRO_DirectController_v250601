import threading
import time
import numpy as np
import gpiozero


class LEDController:
    """
    LED を一元管理するクラス。プリセットを set_preset() で設定すると、
    内部スレッドで「点灯パターン」を更新し続ける。

    プリセット名 (preset) は以下のいずれかを想定：
      - "INIT"     : 初期化フェーズ → 白点滅
      - "AUTO"     : 自動モード → 白常灯
      - "IDLE"     : 初期アイドル → 緑点滅
      - "NORMAL"   : 正常 → 緑点灯（常灯）
      - "ERROR"    : エラー → 黄点灯（赤+緑）
      - "EMC"      : 電源喪失(EMC) → 赤点灯
    """

    def __init__(self, led_r, led_g, led_b, led_cycletime, led_interval):
        self.m_led_r = led_r
        self.m_led_g = led_g
        self.m_led_b = led_b
        self._ledcmd = "IDLE"
        self._lock = threading.Lock()
        self._stop = False

        self.led_cycle = led_cycletime
        self.led_interval = led_interval

    def set_ledcmd(self, ledcmd_name: str):
        """
        プリセット名を外部（他スレッド）から変更する。
        例: "INIT", "IDLE", "NORMAL", "ERROR", "EMC"
        """
        with self._lock:
            self._ledcmd = ledcmd_name

    def get_ledcmd(self) -> str:
        """内部用：現在のプリセットを取得"""
        with self._lock:
            return self._ledcmd

    def stop_led(self):
        """run() 内部ループを止めるフラグを立てる"""
        self._stop = True

    def run_led(self):
        """
        LED 制御用のメインループ。LED_INTERVALごとにプリセットを読み取り、
        PWMLED の value をセットし続ける。
        """
        t_0 = time.time()
        while self._stop != True:
            ledcmd = self.get_ledcmd()
            t_1 = time.time()
            t_elaps = t_1 - t_0
            dutycycle = 0.5 * (
                1 - np.cos(2 * np.pi * (t_elaps % self.led_cycle) / self.led_cycle)
            )

            led_rgb_val = [0.0, 0.0, 0.0]

            if ledcmd == "EMC":
                # 赤常灯
                led_rgb_val = [1.0, 0.0, 0.0]

            elif ledcmd == "ERROR":
                # 黄色常灯
                led_rgb_val = [1.0, 1.0, 0.0]

            elif ledcmd == "IDLE":
                # 緑点滅
                led_rgb_val = [0.0, dutycycle, 0.0]

            elif ledcmd == "NORMAL":
                # 緑常灯
                led_rgb_val = [0.0, 1.0, 0.0]

            elif ledcmd == "AUTO":
                led_rgb_val = [1.0, 1.0, 1.0]

            elif ledcmd == "DIRECTM":
                # 青常灯
                led_rgb_val = [0.0, 0.0, 1.0]

            elif ledcmd == "INIT":
                # 白点滅
                led_rgb_val = [dutycycle, dutycycle, dutycycle]

            else:  # 例外:紫
                led_rgb_val = [1.0, 0.0, 1.0]

            self.m_led_r.value = led_rgb_val[0]
            self.m_led_g.value = led_rgb_val[1]
            self.m_led_b.value = led_rgb_val[2]

            t_2 = time.time()
            if (t_2 - t_1) <= self.led_interval - 0.001:
                time.sleep(self.led_interval - (t_2 - t_1))
            t_3 = time.time()
            # print(f"cycletime:{t_3 - t_1}")

        self.m_led_r.off()
        self.m_led_g.off()
        self.m_led_b.off()
