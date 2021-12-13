from machine import Pin, PWM, Timer
from time import sleep
import math
KEY_UP = const(0)
KEY_DOWN = const(1)

#SmartBox class
class SmartBox:

    def __init__(self):
        # initilize software variables
        # password type string
        self.is_opened = False
        self.user_password = '1234'
        self.password_list = [self.user_password]

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
            while not(key_pressed):
                for row in range(4):
                    for col in range(4):
                        key = scan(row, col)
                        if key == KEY_DOWN:
                            key_pressed = True
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
            return False
        else:
            print('Correct password')
            return True

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

# main controlling system
if __name__ == '__main__':
    print('ok')
    smartbox = SmartBox()
    while True:
        if smartbox.input_password():
            smartbox.open()
        if smartbox.get_single_key_input():
            smartbox.close()
