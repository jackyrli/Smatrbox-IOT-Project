from machine import Pin, PWM,Timer
from time import sleep
import math


class Servo:

    #object of servo motor control.

    def __init__(self,pinnumber):
        #setting up pwm pin and servomotor, takes in a pinnumber.
        self.pinnumber = pinnumber
        self.pin = Pin(self.pinnumber, mode=Pin.OUT)
        self.servo = PWM(self.pin,freq=50,duty=2.5,timer=0)

    def map(self, x, inlow, inhigh, outlow, outhigh):
        return float((x-inlow)*(outhigh-outlow)/(inhigh-inlow)+outlow)

    def demo(self):
        #servo test program,demo of rotation from 0 to 180 deg.
        for deg in range(0,151,30):
            input = self.map(deg,270,0,2,12)
            self.servo.duty(input)
            time.sleep(1)

    def moveto(self,deg):
        #servo moves to certain angle bewtween 0-180 deg.
        input = self.map(deg,270,0,2,12)
        self.servo.duty(input)

#test
servo1 = Servo(27)
servo1.moveto(0)
time.sleep(1)
servo1.moveto(140)
time.sleep(1)
