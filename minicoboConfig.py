from gpiozero import Button, PWMLED, DigitalOutputDevice
from Dynamixel import Dynamixel, Test_Dynamixel

# raspiGPIOピン定義
PIN_BUTTON = 26
PIN_MOTOR_POWER = 16
PIN_FAN_EN = 5
PIN_FAN_IN1 = 6
PIN_LED_RED = 12
PIN_LED_GREEN = 13
PIN_LED_BLUE = 19

# minicoboLEDパラメータ
LED_CYCLETIME = 1.5
LED_INTERVAL = 0.05

# miniCOBOTTAPRO各種パラメータ
"""
OPERATINGMODEのプリセット名は以下のいずれか(Dynamixel e-manual参照)
    - "Current"             : 電流制御モード
    - "Velocity"            : 速度制御モード
    - "Position"            : 位置制御モード
    - "ExtendedPosition"    : 拡張位置制御モード
    - "CurrentBasedPosition": 位置電流制御モード
    - "PWM"                 : PWM制御モード
"""
DXL_IDS = [1, 2, 3, 4, 5, 6]
BAUNDRATE = 2000000
USBPORT = "/dev/ttyUSB0"
OPERATINGMODE = "Position"


class RaspiGPIO:
    def __init__(self):
        self.btn_run = Button(PIN_BUTTON, pull_up=False, bounce_time=0.05)
        self.motor_pw = DigitalOutputDevice(PIN_MOTOR_POWER, initial_value=False)
        self.fan_en = DigitalOutputDevice(PIN_FAN_EN, initial_value=False)
        self.fan_in1 = DigitalOutputDevice(PIN_FAN_IN1, initial_value=False)
        self.led_r = PWMLED(PIN_LED_RED)
        self.led_g = PWMLED(PIN_LED_GREEN)
        self.led_b = PWMLED(PIN_LED_BLUE)

        self.led_cycletime = LED_CYCLETIME
        self.led_interval = LED_INTERVAL


class miniCOBOTTAPRO:
    def __init__(self):
        self.DXL_IDs = DXL_IDS
        self.dxl = Dynamixel(USBPORT, BAUNDRATE)
        if OPERATINGMODE == "Current":
            self.ctrlmode = [0]
        elif OPERATINGMODE == "Velocity":
            self.ctrlmode = [1]
        elif OPERATINGMODE == "ExtendedPosition":
            self.ctrlmode = [4]
        elif OPERATINGMODE == "CurrentBasedPosition":
            self.ctrlmode = [5]
        elif OPERATINGMODE == "PWM":
            self.ctrlmode = [6]
        elif OPERATINGMODE == "Position":
            self.ctrlmode = [3]
        else:
            self.ctrlmode = [3]
