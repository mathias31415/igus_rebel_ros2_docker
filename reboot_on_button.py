import RPi.GPIO as GPIO
import os
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        if GPIO.input(18) == GPIO.LOW:
            os.system('sudo reboot')
            time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()