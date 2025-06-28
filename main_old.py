from gpiozero import Button, PWMLED, DigitalOutputDevice
from signal import pause
from concurrent.futures import ThreadPoolExecutor
import time


button_startstop = Button(
    26, pull_up=False, bounce_time=0.05
)  # GPIO26: タクトスイッチ（LOWで押されたと判断する）
fan_en = DigitalOutputDevice(5, initial_value=False)  # GPIO5: L293D ENピン
fan_in1 = DigitalOutputDevice(6, initial_value=False)  # GPIO6: L293D IN1ピン
mosfet_in = DigitalOutputDevice(16, initial_value=False)
led_r = PWMLED(12)
led_g = PWMLED(13)  # PWM対応
led_b = PWMLED(19)

fan_mosfet_state = False  # 初期状態: OFF
executor = ThreadPoolExecutor(max_workers=1)
animation_future = None
stop_animation = False


def led_breathe():
    """LEDをゆっくり点灯・消灯させ続ける"""
    global stop_animation
    while not stop_animation:
        # 点灯 (0 → 1)
        for i in range(20):
            if stop_animation:
                break
            led_b.value = i / 20
            led_r.value = (i / 20) * 0.6
            time.sleep(0.037)
        # 消灯 (1 → 0)
        for i in range(20):
            if stop_animation:
                break
            led_b.value = 1 - (i / 20)
            led_r.value = (1 - (i / 20)) * 0.6
            time.sleep(0.037)
    led_g.off()


def toggle_output():
    global fan_mosfet_state, animation_future, stop_animation
    fan_mosfet_state = not fan_mosfet_state

    if fan_mosfet_state:
        print("mini COBOTTA PRO debug mode")
        fan_en.on()
        fan_in1.on()
        mosfet_in.on()

        stop_animation = False
        animation_future = executor.submit(led_breathe)

    else:
        print("mini COBOTTA PRO power off")
        fan_en.off()
        fan_in1.off()
        mosfet_in.off()

        stop_animation = True
        if animation_future is not None:
            animation_future.result()  # 完了まで待つ
        led_g.off()
        led_r.off()
        led_b.off()

    time.sleep(0.05)


button_startstop.when_pressed = toggle_output

print("ボタン待機中...（GPIO26）")
pause()
