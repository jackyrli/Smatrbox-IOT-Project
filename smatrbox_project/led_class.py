from board import LED
from machine import Pin, PWM, Timer
from time import sleep

class LED:

    def __init__(self):
        self.led_ext = Pin(32, mode=Pin.OUT)
        self.state = 'alarm'
        self.brightness = 0
        self.L1 = PWM(self.led_ext,freq=500,duty=self.brightness,timer=0)
        self.t1 = Timer(2)
        self.t1.init(period=10, mode=self.t1.PERIODIC, callback=self.tcb)

    def tcb(self, timer):
        if self.state == 'opened':
            self.brightness = 100
        elif self.brightness < 100 and self.state == 'alarm':
            self.brightness += 1
        else:
            self.brightness = 0
        self.L1.duty(self.brightness)

class LED1:
    def __init__(self):
        self.led_ext = Pin(32, mode=Pin.OUT)
    def on(self):
        self.led_ext.value(1)
    def off(self):
        self.led_ext.value(0)

if __name__ == '__main__':
    led = LED1()
    for i in range(30):
        led.on()
        sleep(1)
        led.off()
        sleep(1)
