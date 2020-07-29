'''
Program for Terence with obstacle avoidance on which to build occupancy grid and SLAM experiments.
28 07 20 created basic turn on the spot bumper program.
To Do -
Braitenberg experiment with eight sonar
integrate odometry with rotational PID
Build map
'''
# Import Stuff
import RPi.GPIO as GPIO
import time
import serial

# Serial Setup
bot = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
bot.flush()

# Set up motors and GPIO pins
GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
GPIO.setwarnings(False)
GPIO.setup(22, GPIO.OUT) # Left Motor Forward
GPIO.setup(23, GPIO.OUT) # Left Motor Backward
GPIO.setup(27, GPIO.OUT) # Right Motor Forward
GPIO.setup(18, GPIO.OUT) # Right Motor Backward
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Pause switch
leftFor = GPIO.PWM(22, 50)
leftBac = GPIO.PWM(23, 50)
rightFor = GPIO.PWM(27, 50)
rightBac = GPIO.PWM(18, 50)

# initiate Motors
leftDutyCycle, rightDutyCycle = 0, 0
leftFor.start(leftDutyCycle)
leftBac.start(0)
rightFor.start(rightDutyCycle)
rightBac.start(0)

button = 4 # Pause Switch GPIO 4
pause = 0  # Paused / Resume state
    
# Lets have a function to stop the motors
Stopped = False
def Stop():
    print("STOPPED")
    global leftDutyCycle, rightDutyCycle
    leftDutyCycle, rightDutyCycle = 0, 0
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(0)

def Pause(): # Pause routine, Uses sleep
    global pause
    if GPIO.input(button) == 1:
        time.sleep(.3)
        if pause == 0:
            pause = 1
            Stop()
            print("Paused")
        elif pause == 1:
            pause = 0
            print("Resumed")

dataList = [] # this is where we store the data from the arduino
noData = True # Boolean to let us know if we've received any info from the Arduino

def Serial():  # Communicate with arduino to read encoders, bumpers and sonars
    #print("Serial")50
    try:
        global noData
        bot.write(b"Send\n")
        data = bot.readline().decode('utf-8').rstrip()
        if len(data) < 1:
            print("No data ", data)
            noData = True
        else:
            global dataList
            dataList = data.split(",") # split at comma and make a list
            dataList = list(map(int, dataList)) # Convert string to ints
            noData = False
            #print("DATA", dataList)
            # 0 = Left, 1 = Front and 2 = Right Bumper,
            # 3 = Left Sonar No1, 4 = Left Secondary Sonar No2, 5 = Front Left Secondary Sonar No 3, 6 = Front Left Sonar No 4
            # 7 = Front Right Sonar No5, 8 = Front Right Secondary Sonar No6, 9= Right Secondary Sonar No7, 10 = Right Sonar No 8.
            # 11 = left forward 12 = left back 13 = right forward 14 = right back encoders
    except:
        print('no Connection')
        
def Forward():
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(0)
                    
def SpinLeft():
    leftFor.ChangeDutyCycle(50)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(0)
    rightBac.ChangeDutyCycle(50)
    
def SpinRight():
    leftFor.ChangeDutyCycle(0)
    leftBac.ChangeDutyCycle(50)
    rightFor.ChangeDutyCycle(50)
    rightBac.ChangeDutyCycle(0)

def Sonar(): # Eight Sonar 
    global dataList, leftDutyCycle, rightDutyCycle
    #dataList 3 to 10
    leftDutyCycle = 50 + (dataList[7] - 50)
    rightDutyCycle = 50 + (dataList[6] - 50)
    

# Step/time parameters
lasttime = time.time() # Variable to store time for timesteps
step = 0
previousStep = 0

print("Starting")
time.sleep(1)

while True:

    Pause()
    if pause == 1:
        pass
    else:
        if  time.time() > lasttime + 0.1:#0.05 = 50 milliseconds = 20 Hertz
                lasttime = time.time()
                step = time.time() - previousStep
                previousStep = time.time()
                #print("Iteration time = ", round(step, 4))
                while noData == True:
                    Stop()
                    Serial()
                Serial()
                LB = dataList[0]
                FB = dataList[1]
                RB = dataList[2]
                if LB == 0:
                    SpinLeft()
                    print("Left Hit")
                elif FB == 0:
                    SpinRight()
                    print('Front Hit')
                elif RB == 0:
                    SpinRight()
                    print('Right Hit')
                else:
                    Sonar()
                    print (f'Left Wheel ', {leftDutyCycle}, ', right Wheel ', {rightDutyCycle})
                    Forward()
