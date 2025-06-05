from gpiozero import Button, LED
import time

# 핀 설정 (BCM 기준)
button = Button(25, pull_up=True)
led_pins = [8, 7, 16, 20]

# LED 객체 생성
leds = [LED(pin) for pin in led_pins]

try:
    while True:
        if button.is_pressed:
            # LED 순차적 점등
            for led in leds:
                led.on()
                time.sleep(0.5)
                led.off()
        time.sleep(0.5)  # 스위치 상태 주기적 체크

except KeyboardInterrupt:
    print("프로그램 종료됨")
