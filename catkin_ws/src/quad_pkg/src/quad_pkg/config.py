import board
import busio
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO

# shared varables
kit = None
temp = None

def setup():
    
    # setup GPIOs
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    disable = 17             # OE pin of pwm-board (if set HIGH, PWM disabled)

    # setup I2C bus
    global i2c
    i2c = busio.I2C(board.SCL, board.SDA)

    # setup pca object (pca-Board: VCC-> 3.3V from RPi, GND-> GND from RPi, SCL/SDA-> SCL/SDA from RPi, V+/GND (high Voltage)-> 5V/GND external supply)
    global kit 
    kit = ServoKit(channels=16)
    kit.frequency = 50
    for i in range(2,16):
        kit.servo[i].set_pulse_width_range(500, 2500)