import RPi.GPIO as GPIO
import time
import subprocess
import os
import signal

GPIO.setmode(GPIO.BCM)
button_pin = 22
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Track the running process
process = None

def button_pressed(channel):
    global process
    if process is None:
        print("Button pressed! Starting script...")
        # Start the process in the background
        process = subprocess.Popen(["python3", "Pygame_Servo_Working_V10_6.py"])
    else:
        print("Button pressed! Stopping script...")
        # Kill the running process
        process.terminate()  # Sends SIGTERM
        process.wait()
        process = None

GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_pressed, bouncetime=500)

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    if process:
        process.terminate()
    GPIO.cleanup()

