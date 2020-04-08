import RPi.GPIO as GPIO
from time import sleep, time
from math import pi

##################################################################
#                  Set up GPIO and HBridge                       #
##################################################################

GPIO.setmode(GPIO.BOARD)  #pin naming
GPIO.setwarnings(False)

RIGHT_MOTOR = 32   #ch1
LEFT_MOTOR = 26  #ch2
LEFT_HALLEFFECT = 7
RIGHT_HALLEFFECT = 5
MOSFET_HBRIDGE_CONTROL = 3   #GPIO 8 is default 3v3

count_left = 0   #left
count_right = 0   #right

GPIO.setup(LEFT_MOTOR, GPIO.OUT)  #left side control
GPIO.setup(RIGHT_MOTOR, GPIO.OUT)   #right side control
GPIO.setup(LEFT_HALLEFFECT, GPIO.IN,pull_up_down=GPIO.PUD_UP) #Hall Effect left
GPIO.setup(RIGHT_HALLEFFECT, GPIO.IN,pull_up_down=GPIO.PUD_UP) #Hall Effect right

def up_count_left(channel): #,reset = False):
    global count_left
    #if reset == True:
    #    count_left = 0
    count_left += 1

def up_count_right(channel): #, reset = False):
    global count_right
    #if reset == True:
    #    count_right = 0
    count_right += 1

GPIO.add_event_detect(RIGHT_HALLEFFECT, GPIO.RISING,callback = up_count_right)
GPIO.add_event_detect(LEFT_HALLEFFECT, GPIO.RISING,callback = up_count_left)

radius_tire = 6.1 #cm
circ_tire = 2*pi*radius_tire
global lat_dist
lat_dist = circ_tire/60 #cm per motor rotation

left = GPIO.PWM(LEFT_MOTOR, 50) #frequency in khz T = 1/f A1
right = GPIO.PWM(RIGHT_MOTOR, 50) #  freq for both sides is set to 50 kHz

left.start(7.5)    #Duty cycle, need 2ms forward,1.5 stop,1ms backward
right.start(7.5)    #7.5 = 1.5 ms duty cycle
sleep(1)
GPIO.setup(MOSFET_HBRIDGE_CONTROL, GPIO.OUT)  #Start HBridge

##################################################################
#                    Main Control Functions                      #
##################################################################

def tick_distance(tick):
    distance = round(tick * lat_dist, 4)
    print (distance)
    return distance

def reset_distance():
    count_right = 0
    count_left = 0

def change_value(x=0):
    if x >100 or x< -100:
        raise ValueError("Value is not between -100 and 100")
    elif x < 0:
        pwm = round(7.5 + (2.5*(x)/100)*1.65, 2)
        print ("Backward ", pwm)
        return pwm
    else:
        pwm = round(7.5 + (2.5*(x)/100), 2)
        print ("Forward ", pwm)
        return pwm

def stop():
    left.ChangeDutyCycle(7.5)
    right.ChangeDutyCycle(7.5)
    sleep(1)

def straight(x = 0):
    pwm = change_value(x)
    left.ChangeDutyCycle(pwm)
    right.ChangeDutyCycle(pwm)
    sleep(.2)

def turn(i= 0,k = 0):
    pwm_L = change_value(i)
    pwm_R = change_value(k)
    left.ChangeDutyCycle(pwm_L)
    right.ChangeDutyCycle(pwm_R)
    sleep(.2)
def get_info():
    print ('RIGHT_MOTOR PIN: ',RIGHT_MOTOR, ' at 50 kHz')
    print ('LEFT_MOTOR PIN: ',LEFT_MOTOR,' at 50 kHz')
    print ('LEFT_HALLEFFECT PIN: ',LEFT_HALLEFFECT)
    print ('RIGHT_HALLEFFECT PIN: ',RIGHT_HALLEFFECT)
    print ('MOSFET_HBRIDGE_CONTROL PIN: ',MOSFET_HBRIDGE_CONTROL)
    print ('radius_tire: ',radius_tire,'cm')
    print ('circ_tire: ',circ_tire,'cm')
    print ('lat_dist: ',lat_dist,'cm')
get_info()
