from machine import Pin, PWM, Timer, I2C
from time import sleep
import math
from mqttclient import MQTTClient
from network import WLAN, STA_IF
import network
import time
from binascii import hexlify
from machine import enable_irq, disable_irq, idle

KEY_UP = const(0)
KEY_DOWN = const(1)

#SmartBox class
class SmartBox:

    def __init__(self):
        # initilize software variables
        # password type string
        self.led = LED()
        self.is_opened = False
        self.user_password = '1234'
        self.delivery_guy_password = '2336'
        self.password_list = [self.user_password, self.delivery_guy_password]
        self.interrupt = False

        self.current_weight = 0
        #keypad stuff
        self.key_mapping = [['1','2','3','A'],['4','5','6','B'],['7','8','9','C'],['*','0','#','D']]
        self.row_pins = []
        self.col_pins = []
        self.last_key_press = None

        # create servo motors
        ''' servo 1 is for controlling lid'''
        self.servo1 = Servo(27)
        self.servo1.moveto(0)
        ''' servo 2 is for controlling lock '''

        #initialize_keypad
        self.initialize_keypad()

    def initialize_keypad(self):
        P1 = Pin(26, mode=Pin.OUT) #A0
        P2 = Pin(25, mode=Pin.OUT) #A1
        P3 = Pin(15, mode=Pin.OUT) #A8
        P4 = Pin(33, mode=Pin.OUT) #A9
        self.row_pins = [P1,P2,P3,P4]

        P5 = Pin(17, mode=Pin.IN,pull=Pin.PULL_DOWN)
        P6 = Pin(16, mode=Pin.IN,pull=Pin.PULL_DOWN)
        P7 = Pin(18, mode=Pin.IN,pull=Pin.PULL_DOWN)
        P8 = Pin(19, mode=Pin.IN,pull=Pin.PULL_DOWN)
        self.col_pins = [P5,P6,P7,P8]
        for row in range(0,4):
            for col in range(0,4):
                self.row_pins[row](0)

    def get_single_key_input(self):
        def scan(row, col):
            # scan a particular gpio and return 1 or 0
            self.row_pins[row](1)
            key = None
            if self.col_pins[col].value() == KEY_DOWN:
                key = KEY_DOWN
            if self.col_pins[col].value() == KEY_UP:
                key = KEY_UP
            self.row_pins[row](0)
            return key
        def helper():
            key_pressed = False
            while (not(key_pressed) and not(self.interrupt)):
                for row in range(4):
                    for col in range(4):
                        key = scan(row, col)
                        if key == KEY_DOWN:
                            key_pressed = True
                            self.led.off()
                            sleep(0.5)
                            self.led.on()
                            self.last_key_press = self.key_mapping[row][col]
                            return self.key_mapping[row][col]
        ret = helper()
        sleep(0.5)
        return ret

    def input_password(self):
        self.initialize_keypad()
        print('please input password')
        password_input = ''
        for i in range(0,100):
            cur = self.get_single_key_input()
            if cur != '#' and cur != '*':
                password_input += cur
            if cur == '#':
                break
            if cur == '*':
                password_input = ''
            print(password_input)
        if (password_input not in self.password_list):
            print('Incorrect password')
            return 0
        elif password_input == self.user_password:
            print('Correct password input by owner')
            return 1
        elif password_input == self.delivery_guy_password:
            print('Thank you for the delivery')
            return 2

    def add_password(self, password):
        # input password: string i.e. '12345A'
        self.password_list.append(password)
        print(password + ' is added')

    #change box state

    def open(self):
        if self.is_opened:
            print('the box is already opened!')
        else:
            self.is_opened = True
            '''TODO: add hardware implementation'''
            self.servo1.open()
            print('box opened')

    def close(self):
        if not(self.is_opened):
            print('the box is already closed!')
        else:
            self.is_opened = False
            '''TODO: add hardware implementation'''
            self.servo1.close()
            print('box closed')

    def alarm(self):

        ''' TODO: add hardware implementation'''

    # interface with internet

# Servo class
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
    def open(self):
        self.moveto(140)
        sleep(0.2)
    def close(self):
        self.moveto(0)
        sleep(0.2)

class Communication:


    def __init__(self,location):
        self.wifi = None
        self.connect_wifi(location)
        self.mqtt = [self.connect_adafruit_jacky(), self.connect_adafruit_ricky()]


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

    def connect_adafruit_jacky(self):
        adafruitIoUrl = 'io.adafruit.com'
        adafruitUsername = 'tanjikede'
        adafruitAioKey = 'aio_cHib49bSsbbyWB7U9RHJTpA8oyq9'
        print("Connecting to Adafruit jacky")
        mqtt = MQTTClient(adafruitIoUrl, port='1883', user=adafruitUsername, password=adafruitAioKey)
        time.sleep(0.5)
        print("Connected! jacky")
        return mqtt

    def connect_adafruit_ricky(self):
        adafruitIoUrl = 'io.adafruit.com'
        adafruitUsername = 'ricky_han'
        adafruitAioKey = 'aio_ewUW71P1s2JXZ9VNlMmGsdg77z3P'
        print("Connecting to Adafruit ricky")
        mqtt = MQTTClient(adafruitIoUrl, port='1883', user=adafruitUsername, password=adafruitAioKey)
        time.sleep(0.5)
        print("Connected! ricky")
        return mqtt

    def publish_feed(self, feedName, content):
        #feedName = "tanjikede/feeds/accelerometer"
        self.mqtt[0].publish(feedName, content)
        print("Published {} to {}.".format(content,feedName))

    def publish_feed_ricky(self, feedName, content):
        #feedName = "tanjikede/feeds/accelerometer"
        self.mqtt[1].publish(feedName, content)
        print("Published {} to {}.".format(content,feedName))

class Accelerometer:

    def __init__(self):
        global communication
        communication = Communication('hesse')
        self.acc_feed = "tanjikede/feeds/accelerometer"
        self.i2c = I2C(1,scl=Pin(22),sda=Pin(23),freq=400000)
        for i in range(len(self.i2c.scan())):
        	print(hex(self.i2c.scan()[i]))
        self.i = i
        buff=[0xA0]
        self.i2c.writeto_mem(self.i2c.scan()[i],0x10,bytes(buff))
        self.i2c.writeto_mem(self.i2c.scan()[i],0x11,bytes(buff))
        time.sleep(0.1)

        # set counting and boolean for evaluating if warning is triggered within last 1 minutes.
        self.count = 1
        self.triggered = False

        self.state = True

        #timer
        self.tim = Timer(1)
        self.tim.init(mode=Timer.PERIODIC, period=100)#period in ms
        self.tim.callback(self.accel_callback)

        self.tot_accel = 1

    def accel_callback(self, t):
        global communication
        if self.count == 601:
            print('reset counter')
            self.count = 1
            self.triggered = False
        else:
            self.count += 1
        self.tot_accel = self.get_total_accel_mag()

        if self.tot_accel > 1.5 and self.triggered == False and self.state == True:
            print('warning!')
            communication.publish_feed(self.acc_feed, 'box shaked')
            self.triggered = True

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

# class for weight system
class HX711:
    def __init__(self, pd_sck=5, dout=4, gain=128):
        self.pSCK = Pin(pd_sck, Pin.OUT)
        self.pOUT = Pin(dout, Pin.IN, pull=Pin.PULL_DOWN)
        self.pSCK.value(False)

        self.GAIN = 0
        self.OFFSET = 0
        self.SCALE = 1

        self.time_constant = 0.25
        self.filtered = 0

        self.set_gain(gain);


    def set_gain(self, gain):
        if gain is 128:
            self.GAIN = 1
        elif gain is 64:
            self.GAIN = 3
        elif gain is 32:
            self.GAIN = 2

        self.read()
        self.filtered = self.read()

    def is_ready(self):
        return self.pOUT() == 0

    def read(self):
        # wait for the device being ready
        for _ in range(500):
            if self.pOUT() == 0:
                break
            time.sleep_ms(1)
        else:
            raise OSError("Sensor does not respond")

        # shift in data, and gain & channel info
        result = 0
        for j in range(24 + self.GAIN):
            state = disable_irq()
            self.pSCK(True)
            self.pSCK(False)
            enable_irq(state)
            result = (result << 1) | self.pOUT()

        # shift back the extra bits
        result >>= self.GAIN

        # check sign
        if result > 0x7fffff:
            result -= 0x1000000

        return result

    def read_average(self, times=3):
        sum = 0
        for i in range(times):
            sum += self.read()
        return sum / times

    def read_lowpass(self):
        self.filtered += self.time_constant * (self.read() - self.filtered)
        return self.filtered

    def get_value(self):
        return self.read_lowpass() - self.OFFSET

    def get_units(self):
        return self.get_value() / self.SCALE

    def tare(self, times=15):
        self.set_offset(self.read_average(times))

    def set_scale(self, scale):
        self.SCALE = scale

    def set_offset(self, offset):
        self.OFFSET = offset

    def set_time_constant(self, time_constant = None):
        if time_constant is None:
            return self.time_constant
        elif 0 < time_constant < 1.0:
            self.time_constant = time_constant

    def power_down(self):
        self.pSCK.value(False)
        self.pSCK.value(True)

    def power_up(self):
        self.pSCK.value(False)

    def weight(self):
        self.tare()
        weightKG = self.read()
        impulse = self.get_value()
        print((weightKG+263300)/21600)
        return (weightKG+263300)/21600

        ##a demo of repeatedly reading data
    def weightdemo(self):
        for i in range(1,100):
            self.weight()
            time.sleep(0.01)

    def weight_5_seconds(self):
        weight_cumulated = [0,0,0,0,0]
        for i in range(5):
            weight_cumulated[i] = self.weight()
            sleep(1)
        average = sum(weight_cumulated)/5
        print('average of the 5 weights is: ', average)
        return average

class LED:
    def __init__(self):
        self.led_ext = Pin(32, mode=Pin.OUT)
        self.led_ext.value(1)
    def on(self):
        self.led_ext.value(1)
    def off(self):
        self.led_ext.value(0)
# main controlling system
if __name__ == '__main__':
    #initialize smartbox object
    signal = LED()
    signal.off()
    smartbox = SmartBox()

    #initialize accelerometer and wifi, adafruit feed
    acc = Accelerometer()

    #initialize weightsensor
    weight_feed = "ricky_han/feeds/weight_sensor"
    weightsensor = HX711()
    signal.on()
    #main loop after initialization
    while True:
        signal.on()
        cmd = smartbox.input_password()
        if cmd == 1 or cmd == 2:
            acc.state = False
            signal.off()
            smartbox.open()
        if smartbox.get_single_key_input():
            acc.state = False
            signal.on()
            smartbox.close()
            acc.state = True
            if cmd == 2: # delivery guy
                cur_weight = math.ceil(weightsensor.weight_5_seconds() * 1000)
                smartbox.current_weight = cur_weight - smartbox.current_weight
                communication.publish_feed_ricky(weight_feed, str(smartbox.current_weight))
            elif cmd == 1:
                cur_weight = math.ceil(weightsensor.weight_5_seconds() * 1000)
                smartbox.current_weight = cur_weight
                if smartbox.current_weight < 0:
                    smartbox.current_weight = 0
                print('your current weight is: ', smartbox.current_weight)
