import Driver
import RPi.GPIO as GPIO
from time import sleep, time
from threading import*
import os , sys , string , serial
import mercury
import re
import queue

##################################################################
#                     Initialize empty Qeueus                    #
##################################################################

Lidar_Left_queue = queue.Queue()
Lidar_Middle_queue = queue.Queue()
Lidar_Right_queue = queue.Queue()
Ultrasonic_Left_queue = queue.Queue()
Ultrasonic_Right_queue = queue.Queue()
RFID_queue = queue.Queue()

##################################################################
#         Initialization and Setup for the Robot GPIO            #
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
servo_box = 12

GPIO.setup(leftIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR left
GPIO.setup(rightIR, GPIO.IN,pull_up_down=GPIO.PUD_DOWN) #IR right
GPIO.setup(leftTrig, GPIO.OUT)  #Ultrasonic leftTrig
GPIO.setup(leftEcho, GPIO.IN)   #Ultrasonic leftecho
GPIO.setup(rightTrig, GPIO.OUT)  #Ultrasonic rightTrig
GPIO.setup(rightEcho, GPIO.IN)   #Ultrasonic rightecho
GPIO.setup(servo_lidar, GPIO.OUT)   #Servo Lidar
GPIO.setup(servo_box, GPIO.OUT)   #Servo Box

##################################################################
#                      USB Serial Setup                         #
##################################################################

ser = serial.Serial("/dev/ttyUSB0", baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
ser.write(b'U')

try:
    reader = mercury.Reader("tmr:///dev/ttyUSB1", baudrate = 115200) #setup for RFID
except TypeError:
    sleep(10)
    reader = mercury.Reader("tmr:///dev/ttyUSB1", baudrate = 115200) #setup for RFID
    pass
reader.set_region("NA2")
reader.set_read_plan( [1], "GEN2", bank = ["user"], read_power = 1533)

##################################################################
#                    Setup for the Servos                        #
##################################################################

servoLidar = GPIO.PWM(servo_lidar,50)
servoBox = GPIO.PWM(servo_box,50)
servoLidar.start(7.4)
servoBox.start(7.0)
sleep(1)

##################################################################
#       Functions to Get The Newest Vaule from the Queues        #
##################################################################

def get_RFID_Value(RFID = 0):
    try:
        RFID_List = []
        for i in range(RFID_queue.qsize()):
            RFID_List.append(RFID_queue.get(block=True,timeout = 3)) # block = true:thread will stop until an item is available in queue
            RFID = RFID_List.pop(len(RFID_List)-1)
    except queue.Empty:
        print('no more RFIDs have been received!')
    return RFID

def get_Left_Lidar_Measurement(L=0):
    try:
        Left_List = []
        for i in range(Lidar_Left_queue.qsize()):
            Left_List.append(Lidar_Left_queue.get(block=True,timeout = 10))
            L = Left_List.pop(len(Left_List)-1)
    except queue.Empty:
        print('no more Values have been received!')
    return L

def get_Middle_Lidar_Measurement(M = 0):
    try:
        Middle_List = []
        for i in range(Lidar_Middle_queue.qsize()):
            Middle_List.append(Lidar_Middle_queue.get(block=True,timeout = 10))
            M = Middle_List.pop(len(Middle_List)-1)
    except queue.Empty:
        print('no more Values have been received!')
    return M

def get_Right_Lidar_Measurement(R = 0):
    try:
        Right_List = []
        for i in range(Lidar_Right_queue.qsize()):
            Right_List.append(Lidar_Right_queue.get(block=True,timeout = 10))
            R = Right_List.pop(len(Right_List)-1)
    except queue.Empty:
        print('no more Values have been received!')
    return R

def get_Left_Ultrasonic_Measurement(L = 0):
    try:
        Left_List = [0]
        for i in range(Ultrasonic_Left_queue.qsize()):
            Left_List.append(Ultrasonic_Left_queue.get(block=True,timeout = 1))
            L = Left_List.pop(len(Left_List)-1)
    except queue.Empty:
        print('no more Values have been received!')
    return L

def get_Right_Ultrasonic_Measurement(R = 0):
    try:
        Right_List = [0]
        for i in range(Ultrasonic_Right_queue.qsize()):
            Right_List.append(Ultrasonic_Right_queue.get(block=True,timeout = 1))
            R = Right_List.pop(len(Right_List)-1)
    except queue.Empty:
        print('no more Values have been received!')
    return R

##################################################################
#                       General Functions                        #
##################################################################

def exit (check = 0):
    k = 0
    k += check
    if k > 0:
        print ('bye')
        os._exit(1)

def close_box_lid(still = 6.97):
    servoBox.ChangeDutyCycle(7)
    #sleep(.5)

def open_box_lid(still = 6.97):
    print ("Opening")
    servoBox.ChangeDutyCycle(6.8)
    sleep(2)
    servoBox.ChangeDutyCycle(6.9)
    sleep(10)
    servoBox.ChangeDutyCycle(7.2)
    print ("closing")
    sleep(2)
    servoBox.ChangeDutyCycle(still)
    sleep(1)

def IR_interupt(channel):
    global IRinterupt
#    Stop()
#    Move_Backward(2)
#    Stop()
    print('IR Emergency Stop')
    IRinterupt = True
    #sleep(.5)

def Read_RFID():
    try:
        temp =reader.get_temperature()
        tag = reader.read(timeout = 2000)
        tag_str = str(tag)
        ID = re.search(r"\'([A-Za-z0-9_]+)\'", tag_str)
        MYID = ID.group(1)
        RFID = str(MYID)
    except AttributeError:
        RFID = None
        pass
    return RFID

def get_LidarDist():
    ser.write(b'R')
    data = []
    i=0
    while (i<=14):
        data.append(str(ord(ser.read())))
        i+=1
    try:
        dist = []
        dist_list = data[8:]
        a = 0
        for a in range(4):
            dist.append(str(chr(int(dist_list[a]))))
        dist_out = (int(dist[0])*1000 + int(dist[1])*100 + int(dist[2])*10 + int(dist[3]))
        dist_out = dist_out / 10
    except ValueError or UnboundLocalError:
        dist_out = 9999
        pass
    return dist_out
    ser.flush()

def get_UltraDist (trig, echo):
    if GPIO.input (echo):
        return 100
    distance = 0
    GPIO.output(trig,False)
    sleep(0.05)
    GPIO.output(trig,True)
    sleep(0.05)
    GPIO.output(trig,False)
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

##################################################################
#                        Main Threads                            #
##################################################################

def Ultrasonic():
    while True:
        UltraLeft  = get_UltraDist(leftTrig, leftEcho)
        UltraRight = get_UltraDist(rightTrig, rightEcho)
        if UltraLeft == 100:
#            print ("LEFT ULTRASONIC ERROR")
            UltraLeft  = get_UltraDist(leftTrig, leftEcho)
        elif UltraRight == 100:
#            print ("RIGHT ULTRASONIC ERROR")
            UltraRight = get_UltraDist(rightTrig, rightEcho)
        else:
            Ultrasonic_Left_queue.put(UltraLeft)
            Ultrasonic_Right_queue.put(UltraRight)

def Lidar():
    while True:
        #left
        servoLidar.ChangeDutyCycle(5.4)
        LidarLeft = get_LidarDist()
        Lidar_Left_queue.put(LidarLeft)
        sleep(.1)
        #middle
        servoLidar.ChangeDutyCycle(7.5)
        LidarMiddle = get_LidarDist()
        Lidar_Middle_queue.put(LidarMiddle)
        sleep(.1)
        #right
        servoLidar.ChangeDutyCycle(9.5)
        LidarRight = get_LidarDist()
        Lidar_Right_queue.put(LidarRight)
        sleep(.1)
        #middle
        servoLidar.ChangeDutyCycle(7.5)
        LidarMiddle = get_LidarDist()
        Lidar_Middle_queue.put(LidarMiddle)
        sleep(.1)

def RFID():
    while True:
        RFID = Read_RFID()
        RFID_queue.put(RFID)
        sleep(.2)

##################################################################
#               Creating and Starting Threads                    #
##################################################################

Lidar_thread = Thread(target=Lidar)   #pass the function (without parenthesis)
Ultrasonic_thread = Thread(target=Ultrasonic)
RFID_thread = Thread(target=RFID)

Lidar_thread.start()
sleep(0.2)
Ultrasonic_thread.start()
sleep(0.2)
RFID_thread.start()
sleep(0.2)  #prevent collision

##################################################################
#                        Sensor Check                            #
##################################################################

def Measurement_check(RFID = 0,Lidar_Left = 0 ,Lidar_Middle = 0 ,Lidar_Right = 0 ,Ultrasonic_Left = 0 ,Ultrasonic_Right = 0):
    global IRinterupt
    check = False
    lidar_front_tol = 8
    lidar_tol = 10
    ultra_tol = 10
    if IRinterupt == False and Lidar_Left > lidar_tol and Lidar_Middle > lidar_front_tol and Lidar_Right > lidar_tol: # and Ultrasonic_Left > ultra_tol and Ultrasonic_Right > ultra_tol:
        check = True
    #print ("Check: ",check)
    #print ("IR: ",IRinterupt)
    #print ("RFID: ", RFID)
    #print ("Lidar Left: ", Lidar_Left," Lidar Middle: ", Lidar_Middle," Lidar Right: ",Lidar_Right)
    #print ('Ultra Left: ',Ultrasonic_Left,'    Ultra Right: ',Ultrasonic_Right)
    IRinterupt = False
    return check

##################################################################
#                      Driver Handler                            #
##################################################################

def Stop():
     Driver.stop()

def Move_Forward(increment = 5, speed=37):
    while Driver.tick_distance(Driver.count_left) < increment and Driver.tick_distance(Driver.count_right) < increment:
        Driver.straight(speed)
        if Driver.tick_distance(Driver.count_left) > 0.9 * increment and Driver.tick_distance(Driver.count_right) > 0.9 * increment:
            Stop()
            Driver.reset_distance()
            sleep(.5)
            break

def Move_Backward(increment = 5 ,speed= -28):
    while Driver.tick_distance(Driver.count_left) < increment and Driver.tick_distance(Driver.count_right) < increment:
        Driver.straight(speed)
        if Driver.tick_distance(Driver.count_left) > 0.9 * increment and Driver.tick_distance(Driver.count_right) > 0.9 * increment:
            Stop()
            Driver.reset_distance()
            sleep(.5)
            break

def Right_Turn(turn_90 = 68 ,speed= 35): # 46
    while Driver.tick_distance(Driver.count_left) < turn_90 and Driver.tick_distance(Driver.count_right) < turn_90:
        #Driver.turn(-50,55) # Driver.turn(-55,65)
        Driver.turn(-45,speed) # Driver.turn(-55,65)
        if Driver.tick_distance(Driver.count_right) > turn_90: # and Driver.tick_distance(Driver.count_right) > 0.75 * turn_90:
            Stop()
            Driver.reset_distance()
            sleep(.5)
            break

def Left_Turn(turn_90 = 50,speed= 35):
    while Driver.tick_distance(Driver.count_left) < turn_90 and Driver.tick_distance(Driver.count_right) < turn_90:
        Driver.turn(speed,-45) #Driver.turn(65,-55)
        if Driver.tick_distance(Driver.count_left) > turn_90: #Driver.tick_distance(Driver.count_left) > 0.75 * turn_90 and Driver.tick_distance(Driver.count_right) > 0.9 * turn_90:
            Stop()
            Driver.reset_distance()
            sleep(.5)
            break

def adjust():
     print('-----------------------adjust--------------------------------')
     Stop()
     Move_Backward(2, -28)
     Stop()
#     sleep(2)

#def coordinator(count):
#    RFIDcoords = [(75,137), (262,182),(216,232)]
#    Wallcoords = [(122,0),(122,33),(267,33),(267,264),(161,264),(161,137),(0,137),(0,0)]
#    BUFFER = 30
#    wall_1_len = abs(Wallcoords[7][1] - Wallcoords[6][1]) - BUFFER # 133
#    wall_2_len = abs(Wallcoords[6][0] - Wallcoords[5][0]) #+  BUFFER
#    wall_3_len = abs(Wallcoords[5][1] - Wallcoords[4][1]) +  BUFFER
#    wall_4_len = abs(Wallcoords[4][0] - Wallcoords[3][0]) - BUFFER
#    wall_5_len = abs(Wallcoords[3][1] - Wallcoords[2][1]) - BUFFER
#    wall_6_len = abs(Wallcoords[2][0] - Wallcoords[1][0]) - BUFFER
#    wall_7_len = abs(Wallcoords[1][1] - Wallcoords[0][1]) - BUFFER
#
#    if count == 1:
#        Move_Forward(wall_1_len)
#        Stop()
#        print('coord 1')
#    elif count == 3:
#        Move_Forward(wall_2_len)
#        Stop()
#        print('coord 2')
#    elif count == 5:
#        Left_Turn()
#        Stop()
#        print('coord 3')
#    elif count == 6:
#        Move_Forward(wall_3_len)
#        Stop()
#        print('coord 4')
    #elif count == 7:
    #    Move_Forward(wall_4_len)
    #    Stop()
    #elif count == 8:
    #    Move_Forward(wall_5_len)
    #    Stop()
    #elif count == 9:
    #    Move_Forward(wall_6_len)
    #    Stop()
#    else:
#        pass

GPIO.add_event_detect(leftIR, GPIO.FALLING,callback = IR_interupt, bouncetime=200)
GPIO.add_event_detect(rightIR, GPIO.FALLING,callback = IR_interupt, bouncetime=200)

##################################################################
#                         Main Thread                            #
##################################################################

try:
    #global start_pos
    global IRinterupt
    IRinterupt = False
    start_pos = (0,0)
    count = 0
    Lidar_Left = 0
    Lidar_Middle = 0
    Lidar_Right = 0
    Ultrasonic_Left = 0
    Ultrasonic_Right = 0
    RFID = 0
    tag_1 = 0
    RFIDcoords = [(75,137), (262,182),(216,232)]
    Wallcoords = [(122,0),(122,33),(267,33),(267,264),(161,264),(161,137),(0,137),(0,0)]
    BUFFER = 30
    wall_1_len = abs(Wallcoords[7][1] - Wallcoords[6][1]) - BUFFER # 133
    wall_2_len = abs(Wallcoords[6][0] - Wallcoords[5][0]) #+  BUFFER
    wall_3_len = abs(Wallcoords[5][1] - Wallcoords[4][1]) +  BUFFER
    wall_4_len = abs(Wallcoords[4][0] - Wallcoords[3][0]) - BUFFER
    wall_5_len = abs(Wallcoords[3][1] - Wallcoords[2][1]) - BUFFER
    wall_6_len = abs(Wallcoords[2][0] - Wallcoords[1][0]) - BUFFER
    wall_7_len = abs(Wallcoords[1][1] - Wallcoords[0][1]) - BUFFER

    RFID_TAG_1 = 'E20037F6673ED389C168F64D'
    RFID_TAG_2 = 'E20037F6673F0349C168F70C'
    RFID_TAG_3 = 'E20037F6673F0549C168F714'
    while True:
        RFID = get_RFID_Value(RFID)
        Lidar_Left = get_Left_Lidar_Measurement(Lidar_Left)
        Lidar_Middle = get_Middle_Lidar_Measurement(Lidar_Middle)
        Lidar_Right = get_Right_Lidar_Measurement(Lidar_Right)
        Ultrasonic_Left = get_Left_Ultrasonic_Measurement(Ultrasonic_Left)
        Ultrasonic_Right = get_Right_Ultrasonic_Measurement(Ultrasonic_Right)
        check = Measurement_check(RFID,Lidar_Left,Lidar_Middle,Lidar_Right,Ultrasonic_Left,Ultrasonic_Right)
        #sleep(1)
        print ('count: ', count )
        close_box_lid()
        if count == 0 and check == True:
            count += 1
            #start_pos = (122 - Ultrasonic_Right , 137 - Lidar_Middle)
            #print ('-----------',start_pos,'------------')
        elif count == 0 and check == False:
            sleep(2)
            #raise Exception('Initial Measurements Invalid')
        elif count > 0 and check == False:
            adjust()
        elif count == 1 and check == True:
            print ('forward')
            Move_Forward(wall_1_len)
            Stop()
            print ('coord 1')
            count += 1
        elif count == 2 and check == True and RFID == None and Lidar_Middle > 17:
            Move_Forward(2)
            Stop()
            print ('hello world')
#            sleep(.2)
        elif count == 2 and check == True and RFID == RFID_TAG_1:
 #           Stop()
            print ('<============= MATCH ==============>')
            print ('<=== ',RFID,' ===>')
            print ('<============= MATCH ==============>')
            open_box_lid()
#            sleep(5)
            #Move_Backward(3)
            #Right_Turn()
#            close_box_lid()
            Stop()
            count += 1
        elif count == 3 and check == True and Lidar_Middle < 25: #35
            Move_Backward(2)
            Stop()
        elif count == 3 and check == True and Lidar_Middle > 25:
            Right_Turn()
            count += 1
        elif count == 4 and check == True:
            print('Hallway 2')
            Move_Forward(wall_2_len)
            Stop()
            print ('coord 2')
            count += 1
        elif count == 5 and check == True and Lidar_Middle > 30: #35
            Move_Forward(2)
            Stop()
        elif count == 5 and check == True and Lidar_Middle < 30:
            count += 1
        elif count == 6 and check == True:
            Left_Turn()
            Stop()
            print ('coord 3')
            print ('Left turn')
            count += 1
        elif count == 7 and check == True:
            print ('Hallway 3')
            Move_Forward(wall_3_len)
            Stop()
            print ('coord 4')
            count += 1
        elif count == 8 and check == True and RFID == None and Lidar_Middle > 15:
            Move_Forward(2)
            Stop()
        elif count == 8 and check == True and RFID == RFID_TAG_2:
        #elif count == 6 and check == True and RFID != None and tag_1 == 0:
            Stop()
            print ('<============= MATCH ==============>')
            print ('<=== ',RFID,' ===>')
            print ('<============= MATCH ==============>')
            open_box_lid()
            sleep(5)
            Move_Backward(10)
            Right_Turn()
            close_box_lid()
            Move_Forward(8)
            Stop()
            tag_1 += 1
            count += 1
        elif count == 9 and check == True and RFID == None and Lidar_Middle > 15: #Too the bathroom
            Move_Forward(2)
            Stop()
        elif count == 9 and check == True and RFID == RFID_TAG_3:
            Stop()
            print ('<============= MATCH ==============>')
            print ('<=== ',RFID,' ===>')
            print ('<============= MATCH ==============>')
            open_box_lid()
            sleep(5)
            Move_Backward(3)
            Right_Turn()
            Stop()
            count += 1
except KeyboardInterrupt:
    print(' ')
    print ("cleaning up!")
    Stop()
    exit(1)
    GPIO.cleanup()
    Lidar_thread.join() #join threads back together to exit properly
    Ultrasonic_thread.join()
    RFID_thread.join()
