# LED ON / OFF with Button Press
# from gpiozero import Button, LED
from gpiozero import Button, LED
import time

button = Button(25, pull_up=True)
led_pins = [8, 7, 16, 20]
leds = [LED(pin) for pin in led_pins]

try:
    while True:
        if button.is_pressed:
            for led in leds:
                led.on()
        else:
            for led in leds:
                led.off()
        time.sleep(0.05)

except KeyboardInterrupt:
    print("종료됨")
