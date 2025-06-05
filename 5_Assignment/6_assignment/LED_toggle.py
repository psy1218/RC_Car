from gpiozero import Button, LED
import time
from signal import pause

# 핀 설정 (BCM 번호 기준)
button = Button(25, pull_up=True)  # 스위치 입력 (풀업 설정)
led_pins = [8, 7, 16, 20]          # LED 출력 핀 목록
leds = [LED(pin) for pin in led_pins]

try:
    while True:
        if button.is_pressed:
            # LED ON
            for led in leds:
                led.on()
            time.sleep(0.5)

            # LED OFF
            for led in leds:
                led.off()
            time.sleep(0.5)
        else:
            time.sleep(0.1)

except KeyboardInterrupt:
    print("프로그램 종료됨")
