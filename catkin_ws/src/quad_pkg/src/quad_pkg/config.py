import board
import busio
import adafruit_pca9685
from adafruit_servokit import ServoKit
import RPi.GPIO as GPIO

# shared variables
kit = None
pca = None

def setup():
    
    # setup GPIOs
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    disable = 17             # OE pin of pwm-board (if set HIGH, PWM disabled)

    # setup I2C bus
    global i2c
    i2c = busio.I2C(board.SCL, board.SDA)

    # setup pca object for Servos (pca-Board: VCC-> 3.3V from RPi, GND-> GND from RPi, SCL/SDA-> SCL/SDA from RPi, V+/GND (high Voltage)-> 5V/GND external supply)
    global kit 
    kit = ServoKit(channels=16)
    kit.frequency = 50

    # setup servos
    for i in range(2,13):
        kit.servo[i].set_pulse_width_range(500, 2500)

    servo_A_alpha = kit.servo[2]
    servo_A_beta = kit.servo[3]
    servo_A_gamma = kit.servo[4]

    servo_B_alpha = kit.servo[5]
    servo_B_beta = kit.servo[6]
    servo_B_gamma = kit.servo[7]

    servo_C_alpha = kit.servo[8]
    servo_C_beta = kit.servo[9]
    servo_C_gamma = kit.servo[10]

    servo_D_alpha = kit.servo[11]
    servo_D_beta = kit.servo[12]
    servo_D_gamma = kit.servo[13]

    #setup pca object for LEDs
    global pca
    pca = adafruit_pca9685.PCA9685(i2c)
    pca.frequency = 50

    led_channel_A = config.pca.channels[0]
    led_channel_B = config.pca.channels[1]
    led_channel_C = config.pca.channels[14]
    led_channel_D = config.pca.channels[15]
