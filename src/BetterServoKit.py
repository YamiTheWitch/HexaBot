'''Reimplemented ServoKit class from the CircuitPython library.'''

import time
from board import SCL, SDA
import busio
import adafruit_motor.servo
from adafruit_pca9685 import PCA9685
from pwmio import PWMOut

class ServoKit():
    '''Can drive 2 PCA9865 PWM-drivers at I2C addresses 0x40 and 0x41. 
    
    The 32 servo motors can be accessed through the servo[] field. The first PWM-driver is mapped to servo motors 0-15 and the second PWM-driver to servo motors 16-31.'''
    def __init__(self):
        '''Can drive 2 PCA9865 PWM-drivers at I2C addresses 0x40 and 0x41. 
    
            The 32 servo motors can be accessed through the servo[] field. The first PWM-driver is mapped to servo motors 0-15 and the second PWM-driver to servo motors 16-31.'''
        i2c = busio.I2C(SCL, SDA)
        pca = PCA9685(i2c,address=64)
        pca.frequency = 50
        pca2 = PCA9685(i2c,address=65)
        pca2.frequency = 50
        i = 0
        self.servo =[]
        for channel in pca.channels:
            self.servo[i]=adafruit_motor.servo.Servo(channel, min_pulse=1100, max_pulse=2100)
            i+=1
        for channel in pca2.channels:
            self.servo[i]=adafruit_motor.servo.Servo(channel, min_pulse=1100, max_pulse=2100)
            i+=1

    
def main():
    #GPIO.cleanup()
    #GPIO.setmode(GPIO.BCM)
    #GPIO.setup(7, GPIO.OUT)
    #GPIO.output(7, True)
    
    kit = ServoKit()
    for servo in kit.servo:
        servo.angle=None
        
    #ps4 = PS4Controller()
    #ps4.init()

    print("debug")
    while True:
        #print("?????")
        #angle = input("Angle: ")
        #print(int((ps4.RX_Axis+1)*90))
        #for servo in kit.servo:
        #    servo.angle = int((ps4.RX_Axis+1)*90)
            
        for servo in kit.servo:
            servo.angle = 90
        #for servo in kit.servo:
            #servo.angle=None
        time.sleep(0.1)
    
    
if __name__ == "__main__":
    main()