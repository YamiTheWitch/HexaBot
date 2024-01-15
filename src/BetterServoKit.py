'''Reimplemented ServoKit class from the CircuitPython library.'''

import time
from board import SCL, SDA
import busio
import adafruit_motor.servo
from adafruit_pca9685 import PCA9685
from pwmio import PWMOut
from typing import Optional

class Servo(adafruit_motor.servo.Servo):  # override to allow for float angle values
    def __init__(
        self,
        pwm_out: "PWMOut",
        *,
        actuation_range: int = 180,
        min_pulse: int = 750,
        max_pulse: int = 2250
    ) -> None:
        super().__init__(pwm_out, min_pulse=min_pulse, max_pulse=max_pulse)
        self.actuation_range = actuation_range
        """The physical range of motion of the servo in degrees.

        :type: float
        """
        self._pwm = pwm_out

    @property
    def angle(self) -> Optional[float]:
        """The servo angle in degrees. Must be in the range ``0`` to ``actuation_range``.
        Is None when servo is disabled."""
        if self.fraction is None:  # special case for disabled servos
            return None
        return self.actuation_range * self.fraction

    @angle.setter
    def angle(self, new_angle: Optional[float]) -> None:
        if new_angle is None:  # disable the servo by sending 0 signal
            self.fraction = None
            return
        if new_angle < 0 or new_angle > self.actuation_range:
            raise ValueError("Angle out of range")
        self.fraction = new_angle / self.actuation_range

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
        self.servo =[Servo]*32
        for channel in pca.channels:
            self.servo[i]=Servo(channel, min_pulse=1100, max_pulse=2100)
            i+=1
        for channel in pca2.channels:
            self.servo[i]=Servo(channel, min_pulse=1100, max_pulse=2100)
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
        #for servo in kit.servo:
            #servo.angle=None
        time.sleep(0.1)
    
    
if __name__ == "__main__":
    main()