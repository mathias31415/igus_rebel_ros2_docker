from gpiozero import Button
import os
import time

button = Button(18)

while True:
    if button.is_pressed:
        os.system('sudo reboot')
        time.sleep(1)