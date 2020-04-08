import RPi.GPIO as GPIO
from time import sleep, time
from threading import*
import os , sys , string , serial
import mercury
import re

##################################################################
#                  Set up for the Robot GPIO                     #
##################################################################

GPIO.setmode(GPIO.BOARD)  #pin naming
GPIO.setwarnings(False)

leftTrig = 11
leftEcho = 13
rightTrig = 22
rightEcho = 24
leftIR = 16
rightIR = 18
servo_lidar = 10

GPIO.setup(leftIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR left
GPIO.setup(rightIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR right
GPIO.setup(leftTrig, GPIO.OUT)  #Ultrasonic leftTrig
GPIO.setup(leftEcho, GPIO.IN)   #Ultrasonic leftecho
GPIO.setup(rightTrig, GPIO.OUT)  #Ultrasonic rightTrig
GPIO.setup(rightEcho, GPIO.IN)   #Ultrasonic rightecho
GPIO.setup(servo_lidar, GPIO.OUT)   #Servo Lidar

ser = serial.Serial("/dev/ttyUSB0", baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
ser.write(b'U')

reader = mercury.Reader("tmr:///dev/ttyUSB1", baudrate = 115200) #setup for RFID
reader.set_region("NA2")
reader.set_read_plan( [1], "GEN2", bank = ["user"], read_power = 1533)

servoLidar = GPIO.PWM(servo_lidar,50)
servoLidar.start(7.4)
sleep(1)

def exit (check = 0):
    k = 0
    k += check
    if k > 0:
        print ('bye')
        os._exit(1)

def IR_interupt():
    interupt = True

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
    #left = 0
    #middle = 0
    #right = 0

    #if servo == 1:
    #    left = dist_out
    #elif servo == 2:
    #    middle = dist_out
    #elif servo == 3:
    #    right = dist_out
    #return left, middle, right
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

#def LidarLeft(input):
#    left = input
#def LidarMiddle(input):
#    middle = input
#def LidarRight(input):
#    right = input
#def UltraLeft(input):
#    Uleft = input
#def UltraRight(input):
#    Uright = input

GPIO.add_event_detect(leftIR, GPIO.FALLING,callback = IR_interupt)
GPIO.add_event_detect(rightIR, GPIO.FALLING,callback = IR_interupt)

class Ultrasonic(Thread):
    def run(self):
#        global distleft, distright
        while True:
            sleep(1)
            #global distleft, distright
            distleft  = get_UltraDist(leftTrig, leftEcho)
            distright = get_UltraDist(rightTrig, rightEcho)
            #print (distleft, "  ",  distright)
            if distleft == 100 or distright == 100:
                print ("Ultrasonic Error")
            else:
                UltraLeft = distleft
                UltraRightdist= right
            exit()

class Lidar(Thread):
    def run(self):
#        global left, middle, right
        while True:
            #left
            servoLidar.ChangeDutyCycle(5.4)
            LidarLeft = get_LidarDist()
            #print (dist)
            #get_LidarDist(1)
            sleep(.1)

            #middle
            servoLidar.ChangeDutyCycle(7.4)
            LidarMiddle = get_LidarDist()
            #print (dist)
            #get_LidarDist(2)
            sleep(.1)

            #right
            servoLidar.ChangeDutyCycle(9.5)
            LidarRight = get_LidarDist()
            #print (dist)
            #get_LidarDist(3)
            sleep(.1)

            #middle
            servoLidar.ChangeDutyCycle(7.4)
            LidarMiddle = get_LidarDist()
            #print (dist)
            #get_LidarDist(2)
            sleep(.1)
            exit()

class RFID(Thread):
    def run(self):
        while True:
            getRFID()
            sleep(1)
            exit()

Ultrasonic().start()
sleep(.2)
Lidar().start()
sleep(0.2)
RFID().start()
sleep(0.2)  #prevent collision

