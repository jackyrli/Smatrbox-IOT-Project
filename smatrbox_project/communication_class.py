from mqttclient import MQTTClient
from network import WLAN, STA_IF
import network
import time

from machine import Pin, Timer, I2C
from binascii import hexlify
import math


class Communication:


    def __init__(self,location):
        self.wifi = None
        self.connect_wifi(location)
        self.mqtt = self.connect_adafruit()


    def callback1(self):
        return None
    def connect_wifi(self, location):
        '''location is String: either 'home' or 'hesse' '''

        wlan = WLAN(STA_IF)
        wlan.active(True)

        if location == 'home':
            wlan.connect('2336wifi', 'jiaobaba')
        elif location == 'hesse':
            wlan.connect('ME100-2.4G', '122Hesse', 5000)
        elif location == 'conway':
            wlan.connect('WiFi','3211021970')
        tries = 0
        while not wlan.isconnected() and tries < 30:
            print("Waiting for wlan connection")
            time.sleep(1)
            tries = tries + 1

        if wlan.isconnected():
                print("WiFi connected at", wlan.ifconfig()[0])
        else:
                print("Unable to connect to WiFi")
        self.wifi = wlan

    def connect_adafruit(self):
        adafruitIoUrl = 'io.adafruit.com'
        adafruitUsername = 'tanjikede'
        adafruitAioKey = 'aio_cHib49bSsbbyWB7U9RHJTpA8oyq9'
        print("Connecting to Adafruit")
        mqtt = MQTTClient(adafruitIoUrl, port='1883', user=adafruitUsername, password=adafruitAioKey)
        time.sleep(0.5)
        print("Connected!")
        return mqtt

    def publish_feed(self, feedName, content):
        #feedName = "tanjikede/feeds/accelerometer"
        self.mqtt.publish(feedName, content)
        print("Published {} to {}.".format(content,feedName))

    def subscribe(self, feedName):
        self.mqtt.subscribe(feedName)

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
        self.tim.init(mode=Timer.PERIODIC, period=1000)#period in ms
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

if __name__ == '__main__':
    c = Communication('home')
    feedName = "tanjikede/feeds/accelerometer"
    acc = Accelerometer()
    for i in range(0, 200):
        if acc.tot_accel > 1.5:
            content = "box shaked"
            c.publish_feed(feedName,content)
        time.sleep(1)
