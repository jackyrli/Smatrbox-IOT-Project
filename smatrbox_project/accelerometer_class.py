from machine import Pin, Timer
from machine import I2C
from binascii import hexlify
import time
import math

class Accelerometer:

    def __init__(self):
        self.i2c = I2C(1,scl=Pin(22),sda=Pin(23),freq=400000)
        for i in range(len(self.i2c.scan())):
        	print(hex(self.i2c.scan()[i]))
        self.i = i
        buff=[0xA0]
        self.i2c.writeto_mem(self.i2c.scan()[i],0x10,bytes(buff))
        self.i2c.writeto_mem(self.i2c.scan()[i],0x11,bytes(buff))
        time.sleep(0.1)

        #timer
        self.tim = Timer(1)
        self.tim.init(mode=Timer.PERIODIC, period=10)#period in ms
        self.tim.callback(self.accel_callback)

        self.tot_accel = 1
    def accel_callback(self, t):
        self.tot_accel = self.get_total_accel_mag()
        if self.tot_accel > 1.5:
            print('warning!')

    def WHOAMI(self, i2caddr):
    	whoami = self.i2c.readfrom_mem(i2caddr,0x0F,1)

    def Temperature(self,i2caddr):
    	temperature = self.i2c.readfrom_mem(i2caddr,0x20,2)
    	if int.from_bytes(temperature,"little") > 32767:
    		temperature = int.from_bytes(temperature,"little")-65536
    	else:
    		temperature = int.from_bytes(temperature,"little")
    	return ((temperature)/(256) + 25)

    def Zaccel(self, i2caddr):
    	zacc = int.from_bytes(self.i2c.readfrom_mem(i2caddr,0x2C,2),"little")
    	if zacc > 32767:
    		zacc = zacc -65536
    	return (zacc/16393)

    def Xaccel(self, i2caddr):
    	xacc = int.from_bytes(self.i2c.readfrom_mem(i2caddr,0x28,2),"little")
    	if xacc > 32767:
    		xacc = xacc -65536
    	return (xacc/16393)

    def Yaccel(self, i2caddr):
    	yacc = int.from_bytes(self.i2c.readfrom_mem(i2caddr,0x2A,2),"little")
    	if yacc > 32767:
    		yacc = yacc -65536
    	return (yacc/16393)

    def get_temperature(self):
        return self.Temperature(self.i2c.scan()[self.i])

    def get_x(self):
        return self.Xaccel(self.i2c.scan()[self.i])

    def get_y(self):
        return self.Yaccel(self.i2c.scan()[self.i])

    def get_z(self):
        return self.Zaccel(self.i2c.scan()[self.i])

    def get_total_accel_mag(self):
        return math.sqrt(self.get_x() ** 2 + self.get_y() ** 2 + self.get_z() ** 2)

# if __name__ == '__main__':
#     acc = Accelerometer()
#
#     while True:
#         a=1
