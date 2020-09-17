'''
program to further test simple pid with rotation. Based on PID_motor
'''

# Import Stuff
import RPi.GPIO as GPIO
import time
import serial
import math
from simple_pid import PID
import matplotlib.pyplot as plt

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
leftBac.start(leftDutyCycle)
rightFor.start(rightDutyCycle)
rightBac.start(rightDutyCycle)

button = 4 # Pause Switch GPIO 4
pause = 0  # Paused / Resume state

# Lets have a function to stop the motors
Stopped = False
def Stop():
    print("STOPPED")
    global leftDutyCycle, rightDutyCycle
    leftDutyCycle, rightDutyCycle = 0, 0
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(leftDutyCycle)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(rightDutyCycle)

def Pause(): # Pause routine, Uses sleep
    global pause, stateFeed
    if GPIO.input(button) == 1:
        time.sleep(.3)
        if pause == 0:
            pause = 1
            Stop()
            Graph()
            print("Paused")
        elif pause == 1:
            pause = 0
            print("Resumed")

PID_data = {'t': [], 'LT': [], 'RT': [], 'LDC': [], 'RDC': []}

def Graph():
    title = ('simple-pid experiments')
    plt.figure(figsize=(12, 8))
    plt.plot(PID_data['t'], PID_data['LT'], label="Left Ticks")
    plt.plot(PID_data['t'], PID_data['RT'], label="Right Ticks")
    plt.plot(PID_data['t'], PID_data['LDC'], label="Left Duty Cycle")
    plt.plot(PID_data['t'], PID_data['RDC'], label="Right Duty Cycle")
    plt.xlabel('Time')
    plt.ylabel('Ticks')
    plt.title(title)
    plt.legend(loc=4)
    plt.show()
    
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
            # 3 = Left Sonar No 1, 4 = Left Secondary Sonar No 2, 5 = Front Left Secondary Sonar No 3, 6 = Front Left Sonar No 4
            # 7 = Front Right Sonar No 5, 8 = Front Right Secondary Sonar No 6, 9= Right Secondary Sonar No 7, 10 = Right Sonar No 8.
            # 11 = left forward 12 = left back 13 = right forward 14 = right back encoders
    except:
        print('no Connection')

def Forward():
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(0)
                    
def Reverse():
    leftFor.ChangeDutyCycle(0)
    leftBac.ChangeDutyCycle(leftDutyCycle)
    rightFor.ChangeDutyCycle(0)
    rightBac.ChangeDutyCycle(rightDutyCycle)

def SpinLeft():
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(0)
    rightBac.ChangeDutyCycle(rightDutyCycle)
    
def SpinRight():
    leftFor.ChangeDutyCycle(0)
    leftBac.ChangeDutyCycle(leftDutyCycle)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(0)    

def TurnLeft():
    leftFor.ChangeDutyCycle(0)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(rightDutyCycle)
    rightBac.ChangeDutyCycle(0)

def TurnRight():
    leftFor.ChangeDutyCycle(leftDutyCycle)
    leftBac.ChangeDutyCycle(0)
    rightFor.ChangeDutyCycle(0)
    rightBac.ChangeDutyCycle(0)      

# Step/time parameters
lasttime = time.time() # Variable to store time for timesteps
step = 0
previousStep = 0
t = 0
  
print("Start")
print("setting up Serial")
time.sleep(1)
print("Getting Data")
while noData == True:
    Serial()
print("DATA", dataList)

# Movement variables
velocity = 0
bearing = 180
rotation = 0
rotationAccuracy = 2

# PID Gubbins
LTPI, RTPI = velocity, velocity # Ticks per Interval, initial setpoint
#tunings = (2, 1.0, 0.01) # Fast but permissibly erratic
#tunings = (1.1, 0.5, 0.5) # makes a bit of a waddle
#tunings = (1.3, 0.7, 0.075) # a little slower to converge but steadiest
tunings = (1.0, 0.5, 0.05) # Default
leftMotor_PID = PID(1.0, 0.5, 0.05, setpoint=LTPI)
rightMotor_PID = PID(1.0, 0.5, 0.05, setpoint=RTPI)
rotational_PID = PID(1.0, 0.5, 0.05, setpoint=0)
leftMotor_PID.tunings = tunings
rightMotor_PID.tunings = tunings
rotational_PID.tunings = tunings
leftMotor_PID.sample_time = 0.01  # update every 0.01 seconds
rightMotor_PID.sample_time = 0.01
rotational_PID.sample_time = 0.01
leftMotor_PID.output_limits = (-100, 100)    # output value (Duty Cycle)
rightMotor_PID.output_limits = (-100, 100)
rotational_PID.output_limits = (-100, 100)

# Odometry variables
prev_leftEncF, prev_leftEncB = 0, 0
prev_rightEncF, prev_rightEncB = 0, 0

#Rotational PID will need some odoemtry in order to work out rotation
travel, TotalTravel, thetaRad, theta = 0, 0, 0, 0
botX, botY = 0, 0

def Odometry():
    wheelbase = 198
    wheelRadius = 41.5
    CPR = 990  # Clicks per Rotation
    wheelc = 2*wheelRadius*math.pi
    mmPC = wheelc/CPR # milimetres per count
    #print(wheelc, mmPC)
    global travel, TotalTravel, thetaRad, theta # theta is direction in which bot is pointing
    global botX, botY
    global leftTicks, rightTicks
    global dataList
    global prev_leftEncF, prev_leftEncB, prev_rightEncF, prev_rightEncB
    leftEncF = dataList[11]
    leftEncB = dataList[12]
    rightEncF = dataList[13]
    rightEncB = dataList[14]
    # Update wheel Ticks
    leftTicks = (leftEncF - prev_leftEncF) - (leftEncB - prev_leftEncB)
    rightTicks = (rightEncF - prev_rightEncF) - (rightEncB - prev_rightEncB)
    prev_leftEncF = leftEncF
    prev_leftEncB = leftEncB
    prev_rightEncF = rightEncF
    prev_rightEncB = rightEncB
    print(leftTicks, ' Left Ticks | Right Ticks ', rightTicks)
    # Update x y and Theta
    leftTravel = leftTicks * mmPC
    rightTravel = rightTicks * mmPC
    travel = (leftTravel + rightTravel)/2
    TotalTravel += travel
    thetaRad += (leftTravel - rightTravel)/wheelbase
    theta = thetaRad*(180/math.pi) #convert to heading in degrees;
    theta -= int(theta/360) * 360 # clip theta to plus or minus 360 degrees
    botX += travel * math.sin(thetaRad);
    botY += travel * math.cos(thetaRad);
    print(int(botX), int(botY), int(theta))

while True:

    Pause()
    if pause == 1:
        pass
    else:
        if  time.time() > lasttime + 0.1: # 0.05 = 75 millis = 13.3 Hertz - 50 milliseconds = 20 Hertz
            lasttime = time.time()
            step = time.time() - previousStep
            #print(step)
            previousStep = time.time()
            t += 1
            while noData == True:
                Stop()
                Serial()
            Serial()
            Odometry()
            # Check Bumpers
            LB = dataList[0]
            FB = dataList[1]
            RB = dataList[2]
            if LB == 0 or FB == 0 or RB == 0: # if bumpers are hit, Stop.
                if Stopped == False:
                    Stop()
                    Graph()
                    Stopped = True                    
            else:
                Stopped = False
                # PID pass commands to motors
                '''
                leftMotor_PID.setpoint = LTPI # motor_PID setpoints set Ticks per interval for speed
                rightMotor_PID.setpoint = RTPI
                leftDutyCycle = leftMotor_PID(leftTicks)
                rightDutyCycle = rightMotor_PID(rightTicks)
                '''
                # Work out something here
                rotationError = bearing - theta
                if rotationError > rotationAccuracy:
                    pass
                else:
                    rotationError = 0
                print('Rotation Error = ', rotationError)
                rotation = rotational_PID(rotationError)
                leftMotor_PID.setpoint = velocity - rotation # formerly LTPI
                rightMotor_PID.setpoint = velocity + rotation # fromerly RTPI
                leftDutyCycle = leftMotor_PID(leftTicks)
                rightDutyCycle = rightMotor_PID(rightTicks)
                print(leftDutyCycle, ' Duty Cycles ', rightDutyCycle)
                # Move the motors
                # Minus values move the wheels backwards and positive values forward.
                if leftDutyCycle < 0:
                    leftFor.ChangeDutyCycle(0)
                    leftBac.ChangeDutyCycle(-leftDutyCycle)
                else:
                    leftFor.ChangeDutyCycle(leftDutyCycle)
                    leftBac.ChangeDutyCycle(0)
                if rightDutyCycle < 0:
                    rightFor.ChangeDutyCycle(0)
                    rightBac.ChangeDutyCycle(-rightDutyCycle)
                else:
                    rightFor.ChangeDutyCycle(rightDutyCycle)
                    rightBac.ChangeDutyCycle(0)
                
                #Act
                #Forward()
                # add some data to a dictionary
                PID_data['t'].append(t)
                PID_data['LT'].append(leftTicks)
                PID_data['RT'].append(rightTicks)
                PID_data['LDC'].append(leftDutyCycle)
                PID_data['RDC'].append(rightDutyCycle)

                '''
                time
                Serial - info from arduino - Get State
                odometry - where t f are we - Get state
                Sensors - react to bumps etc - Vehicle 1
                Decisions/Thinking - Policy
                PID - motor control - Act
                Reward?
                '''
