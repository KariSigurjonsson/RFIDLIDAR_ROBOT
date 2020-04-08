import RPi.GPIO as GPIO
from time import sleep, time
from math import pi
from threading import*
import os , sys , string , serial
import mercury
import re

##################################################################
#                  Set up for the Robot GPIO                     #
##################################################################

GPIO.setmode(GPIO.BOARD)  #pin naming
GPIO.setwarnings(False)

rightMotor = 32   #ch1
leftMotor = 26  #ch2
leftHall = 7
rightHall = 5
leftTrig = 11
leftEcho = 13
rightTrig = 22
rightEcho = 24
leftIR = 16
rightIR = 18
servo_lidar = 10
servo_box = 8
Mosfet = 3   #GPIO 8 is default 3v3

GPIO.setup(leftMotor, GPIO.OUT)  #left side control
GPIO.setup(rightMotor, GPIO.OUT)   #right side control
GPIO.setup(leftHall, GPIO.IN,pull_up_down=GPIO.PUD_UP) #Hall Effect left
GPIO.setup(rightHall, GPIO.IN,pull_up_down=GPIO.PUD_UP) #Hall Effect right
GPIO.setup(leftIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR left
GPIO.setup(rightIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR right
GPIO.setup(leftTrig, GPIO.OUT)  #Ultrasonic leftTrig
GPIO.setup(leftEcho, GPIO.IN)   #Ultrasonic leftecho
GPIO.setup(rightTrig, GPIO.OUT)  #Ultrasonic rightTrig
GPIO.setup(rightEcho, GPIO.IN)   #Ultrasonic rightecho
GPIO.setup(servo_lidar, GPIO.OUT)   #Servo Lidar
#GPIO.setup(servo_Box, GPIO.OUT)   #Servo Lidar

##################################################################
#                          Initialization                        #
##################################################################

radius_tire = 2.4 #inches
circ_tire = 2*pi*radius_tire
dist_tire = circ_tire/60 #inches per motor rotation

count_left = 0   #left
count_right = 0   #right

ser = serial.Serial("/dev/ttyUSB0", baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
ser.write(b'U')

reader = mercury.Reader("tmr:///dev/ttyUSB1", baudrate = 115200) #setup for RFID
reader.set_region("NA2")
reader.set_read_plan( [1], "GEN2", bank = ["user"], read_power = 1533)

left = GPIO.PWM(leftMotor, 50) #frequency in khz T = 1/f A1
right = GPIO.PWM(rightMotor, 50) #  freq for both sides is set to 50 kHz
servoLidar = GPIO.PWM(servo_lidar,50)

print ("The wheel circumference is ",circ_tire)
print ("The 1/60 of circ is ",dist_tire)
print ("stopping wheels")

left.start(7.5)    #Duty cycle, need 2ms forward,1.5 stop,1ms backward
right.start(7.5)    #7.5 = 1.5 ms duty cycle
servoLidar.start(7.4)
sleep(1)

##################################################################
#                    Turn on the Hbridge                         #
##################################################################
GPIO.setup(Mosfet, GPIO.OUT)  #Start HBridge

def exit (check = 0):
    k = 0
    k += check
    if k > 0:
        print ('bye')
        os._exit(1)

def getRFID():
    try:
        temp = reader.get_temperature()
        tag = reader.read(timeout = 1000)
        tag_str = str(tag)
        ID = re.search(r"\'([A-Za-z0-9_]+)\'", tag_str)
        print ("ID: ", ID.group(1), ", temp: " , temp )
    except AttributeError:
        #print ("Passing RFID")
        pass

def get_LidarDist():
    ser.write(b'R')
    data = []  # establishes an empty list to hold all the incoming data
    i=0     # beginning of the buffer
    while (i<=14):
        data.append(str(ord(ser.read()))) #adding each incoming bit to the list
        i+=1
    try:
        #establishing an output / gd = good distance
        dist = []
        dist_list = data[8:] # trying to separate the unnecessary parts of the data
        a = 0
        for a in range(4):
            dist.append(str(chr(int(dist_list[a]))))
        dist_out = int(dist[0])*1000 + int(dist[1])*100 + int(dist[2])*10 + int(dist[3])
    except ValueError or UnboundLocalError:
        pass
    return dist_out
    ser.flush()

def get_UltraDist (trig, echo):
    if GPIO.input (echo):
        return 100

    distance = 0
    GPIO.output (trig,False)
    sleep(0.05)
    GPIO.output (trig,True)
    sleep(0.01)
    GPIO.output (trig,False)
    time1, time2 = time(), time()

    while not GPIO.input(echo):
        time1 = time()
        if time1 - time2 > 0.02:      #pin doesn't go low after 20 ms set dist to 100
            distance = 100
            break

    if distance == 100:
        return distance

    while GPIO.input(echo):
        time2 = time()
        if time2 - time1 > 0.02:      #pin doesn't go high after 20 ms set dist to 100
            distance = 100
            break

    distance = (time2 - time1) / 0.00000295 / 2 / 10
    return distance

def get_Halleffect_Dist(tick):
    global dist_tire
    dist = tick * dist_tire
    return dist

def stop():
    print ("stopping")
    left.ChangeDutyCycle(7.5)
    right.ChangeDutyCycle(7.5)
    sleep(1)

def stopIR(channel):
    print ("IR stop")
    left.ChangeDutyCycle(7.5)
    right.ChangeDutyCycle(7.5)
    sleep(1)

def forward():
    print ("Moving forward")
    left.ChangeDutyCycle(10)
    right.ChangeDutyCycle(10)
    sleep(1)
    stop()

def leftturn():
    print ("Turning Left")
    left.ChangeDutyCycle(10)
    right.ChangeDutyCycle(5)
    sleep(.5)
    stop()

def rightturn():
    print ("Turning Right")
    left.ChangeDutyCycle(5)
    right.ChangeDutyCycle(10)
    sleep(.5)
    stop()

def backward():
    print ("Backing")
    left.ChangeDutyCycle(5)
    right.ChangeDutyCycle(5)
    sleep(1)
    stop()

def slow():
    print ("test")
    x = 0
    while x < 15:
        left.ChangeDutyCycle(x)
        right.ChangeDutyCycle(x)
        sleep(.2)
        print (x)
        x += .1
    stop()

def arch():
    print ("one turn")
    left.ChangeDutyCycle(8.5)
    right.ChangeDutyCycle(10)
    sleep(2)
    stop()

def up_count_left(channel):
    global count_left
    count_left += 1

def up_count_right(channel):
    global count_right
    count_right += 1

GPIO.add_event_detect(leftIR, GPIO.FALLING,callback = stopIR)
GPIO.add_event_detect(rightIR, GPIO.FALLING,callback = stopIR)
GPIO.add_event_detect(rightHall, GPIO.RISING,callback = up_count_right)
GPIO.add_event_detect(leftHall, GPIO.RISING,callback = up_count_left)

class Drive(Thread):    #need Thread for the thread library
    def run(self):      #needs to be called run cuz its in the library
        try:
            while True:
                userinput = input("Command?\n")
                if userinput == "x":
                    stop()
                if userinput == "w":
                    forward()             ### multi comment cmd + /
                if userinput == "d":
                    rightturn()
                if userinput == "a":
                    leftturn()
                if userinput == "s":
                    backward()
                if userinput == "2":
                    slow()
                if userinput == "q":
                    arch()
                if userinput == "exit":
                    exit(1)

        except KeyboardInterrupt:
            print ()
            print ('hard exit')
            GPIO.cleanup()
        GPIO.cleanup()

class HallEffect(Thread):
    def run(self):
        global rightHall
        global leftHall
        while True:
            checkL = count_left
            checkR = count_right
            sleep(.5)
            if count_left == checkL or count_right == checkR:
                sleep(.1)
            else:
                L = get_Halleffect_Dist(count_left)
                R = get_Halleffect_Dist(count_right)
                print (L ," and ", R)
            exit()

class Ultrasonic(Thread):
    def run(self):
        while True:
            sleep(1)
            distleft  = get_UltraDist(leftTrig, leftEcho)
            distright = get_UltraDist(rightTrig, rightEcho)
            #print (distleft, "  ",  distright)
            if distleft == 100 or distright == 100:
                print ("Ultrasonic not Reading")
            elif distleft < 5.0 or distright < 5.0:
                print ("Slowing Down")
                stop()
            exit()

class Lidar(Thread):
    def run(self):
        while True:
            #left
            servoLidar.ChangeDutyCycle(5.4)
            dist = get_LidarDist()
            print (dist)
            sleep(1)

            #middle
            servoLidar.ChangeDutyCycle(7.4)
            dist = get_LidarDist()
            print (dist)
            sleep(1)

            #right
            servoLidar.ChangeDutyCycle(9.5)
            dist = get_LidarDist()
            print (dist)
            sleep(1)

            #middle
            servoLidar.ChangeDutyCycle(7.4)
            dist = get_LidarDist()
            print (dist)
            sleep(1)
            exit()

class RFID(Thread):
    def run(self):
        while True:
            getRFID()
            sleep(1)
            exit()

drive_obj = Drive()
hall_obj = HallEffect()
ultra_obj = Ultrasonic()
lidar_obj = Lidar()
rfid_obj = RFID()

hall_obj.start()
sleep(.2)
ultra_obj.start()
sleep(.2)
lidar_obj.start()
sleep(0.2)
rfid_obj.start()
sleep(0.2)
drive_obj.start()
sleep(0.2)   #prevent collision
