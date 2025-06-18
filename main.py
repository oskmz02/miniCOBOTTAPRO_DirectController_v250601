from concurrent.futures import ThreadPoolExecutor
import queue
import time

from minicoboConfig import RaspiGPIO, miniCOBOTTAPRO
from minicoboSystemControl import SystemControlThread
from minicoboLED import LEDController
from minicoboDirver import MiniCoboThread


def main():
    # Configインスタンス化
    rp = RaspiGPIO()
    mc = miniCOBOTTAPRO()

    led_ctrl = LEDController(
        rp.led_r, rp.led_g, rp.led_b, rp.led_cycletime, rp.led_interval
    )

    # Queue定義
    cmd_q = queue.Queue()
    cmd_minicobo_q = queue.Queue()
    st_minicobo_q = queue.Queue()

    executor = ThreadPoolExecutor(max_workers=3)

    led_ftr = executor.submit(led_ctrl.run_led)

    rp.motor_pw.on()
    print(f"[MAIN] Motor Power ON")

    # miniCOBOTTAPRO初期セットアップ
    led_ctrl.set_ledcmd("INIT")
    mcth = MiniCoboThread(
        st_minicobo_q,
        cmd_minicobo_q,
        mc.dxl,
        mc.DXL_IDs,
        mc.ctrlmode,
    )
    while mcth.st_minicobo_init != True:
        mcth.startUpSequence(mc.dxl, mc.DXL_IDs, mc.ctrlmode)
        time.sleep(2)

    led_ctrl.set_ledcmd("IDLE")

    systh = SystemControlThread(rp.btn_run, cmd_q)
    sys_ftr = executor.submit(systh.run)
    print(f"[MAIN] System all green")
    print(f"\n[MAIN] Push START/STOP BUTTON to start operation")

    st_minicobo_run = False
    mc_ftr = None
    t_stdw = time.time()
    pad_ = 0

    try:
        while True:
            pad_ += 1
            t_st = time.time()
            try:
                cmd = cmd_q.get_nowait()
            except queue.Empty:
                cmd = None

            if cmd == "TOGGLE":
                if st_minicobo_run != False:
                    cmd_minicobo_q.put("STOP")
                    st_minicobo_run = False
                    t_stdw = time.time()
                    # if t_stdw - time.time()
                    rp.fan_en.off()
                    rp.fan_in1.off()
                    print(f"[MAIN] STOPPING miniCOBOTTAPRO")
                else:
                    print(f"[MAIN] STARTING miniCOBOTTAPRO")
                    rp.fan_en.on()
                    rp.fan_in1.on()
                    if mc.ctrlmode == [3]:
                        mc_ftr = executor.submit(
                            mcth.directControlPos, mc.dxl, mc.DXL_IDs
                        )
                    else:
                        mc_ftr = executor.submit(
                            mcth.directControlCurPos, mc.dxl, mc.DXL_IDs
                        )
                    st_minicobo_run = True
            try:
                st = st_minicobo_q.get_nowait()
            except queue.Empty:
                st = None

            if st != None:
                led_ctrl.set_ledcmd(st)

            if (
                st_minicobo_run != True
                and pad_ % 120 == 0
                and (time.time() - t_stdw) > 5
            ):
                mc.dxl.readPresentPosition(mc.DXL_IDs)
                if all(mc.dxl.getdata_result_array) != True:
                    print(f"com_err")
                    mcth.startUpSequence(mc.dxl, mc.DXL_IDs, mc.ctrlmode)

            t_now = time.time()
            if (t_now - t_st) <= 0.007:
                time.sleep(0.008 - (t_now - t_st))
            # time.sleep(0.008)
    except KeyboardInterrupt:
        print("\n[MAIN] System Shutdown...")

    systh.stop()
    if sys_ftr != None:
        sys_ftr.result()
    if (st_minicobo_run == True) and (mc_ftr != None):
        cmd_minicobo_q.put("STOP")
        mc_ftr.result()
    led_ctrl.stop_led()
    if led_ftr != None:
        led_ftr.result()

    rp.motor_pw.off()
    print(f"[MAIN] Exit complete")


if __name__ == "__main__":
    main()
