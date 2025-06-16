import queue
import threading
import time

# from minicoboconfig import RaspiGPIO


class SystemControlThread:
    """
    START/STOPボタンを監視し、"TOGGLE" コマンドを cmd_queue に流すクラス。
    """

    def __init__(self, btn_run, cmd_q: queue.Queue):
        self.m_btn_run = btn_run
        self.m_cmd_q = cmd_q
        self.m_stop_evt = threading.Event()

    def _on_btn_run(self):
        """
        ボタンが押されたときにコールバックされる関数。
        キューに "TOGGLE" を追加する。
        """
        self.m_cmd_q.put("TOGGLE")

    def run(self):
        """
        gpiozero の when_pressed にコールバックを登録し、
        イベントがセットされるまでループして待機
        """
        self.m_btn_run.when_pressed = self._on_btn_run

        while self.m_stop_evt.is_set() != True:
            time.sleep(0.1)

    def stop(self):
        """外部から停止要求が来たらイベントをセットする"""
        self.m_stop_evt.set()
