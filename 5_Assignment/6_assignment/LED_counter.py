from gpiozero import Button, LED
import time

# 핀 번호 설정
button = Button(25, pull_up=True)
led_pins = [8, 7, 16, 20]  # bit0 ~ bit3
leds = [LED(pin) for pin in led_pins]

try:
    while True:
        if button.is_pressed:
            for i in range(16):  # 0 ~ 15
                for bit in range(4):
                    # 각 비트에 해당하는 LED ON/OFF
                    # 0 ~ 3 비트에 해당하는 LED 점등
                    # i >> (3 - bit) & 1 :  # i의 3 - bit 번째 비트가 1이면 ON, 아니면 OFF
                    if (i >> (3 - bit)) & 1:
                        leds[bit].on()
                    else:
                        leds[bit].off()
                time.sleep(0.5)

            # 모든 LED OFF
            for led in leds:
                led.off()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("종료됨")
