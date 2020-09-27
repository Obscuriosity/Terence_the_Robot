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
    leftDutyCycle, rightDutyCycle = 0, 0 # Or velocity = 0 / setpoint 0
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

PID_data = {'t': [], 'A': [], 'B': [], 'C': [], 'D': []}

def Graph():
    title = ('Odometry experiments')
    plt.figure(figsize=(12, 8))
    #plt.plot(PID_data['t'], PID_data['A'], label="Theta")
    #plt.plot(PID_data['t'], PID_data['B'], label="Bearing")
    #plt.plot(PID_data['t'], PID_data['C'], label="velocity")
    #plt.plot(PID_data['t'], PID_data['D'], label="rotation")
    plt.scatter(PID_data['A'], PID_data['B'])
    plt.xlabel('x')
    plt.ylabel('Y')
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
    global velocity, rotation
    #velocity = velocity
    rotation = 0
                    
def Reverse():
    global velocity, rotation
    #velocity = velocity
    rotation = 0

def SpinLeft():
    global velocity, rotation
    rotation = -velocity
    velocity = 0
    
def SpinRight():
    global velocity, rotation
    rotation = velocity
    velocity = 0

def TurnLeft(): # Left wheel stationary right moves
    global velocity, rotation
    rotation = velocity
    velocity = rotation

def TurnRight(): # Right wheel stationary left moves
    global velocity, rotation
    rotation = velocity
    velocity = -rotation

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
bearing = 90
rotation = 0
rotationAccuracy = 1

# PID Gubbins
LTPI, RTPI = velocity, velocity # Ticks per Interval, initial setpoint
#tunings = (2, 1.0, 0.01) # Fast but permissibly erratic
#tunings = (1.1, 0.5, 0.5) # makes a bit of a waddle
#tunings = (1.3, 0.7, 0.075) # a little slower to converge but steadiest
tunings = (1.0, 0.5, 0.05) # Default
rotationalTunings = (0.2, 0.1, 0.06)
leftMotor_PID = PID(1.0, 0.5, 0.05, setpoint=LTPI)
rightMotor_PID = PID(1.0, 0.5, 0.05, setpoint=RTPI)
rotational_PID = PID(1.0, 0.5, 0.05, setpoint=0)
leftMotor_PID.tunings = tunings
rightMotor_PID.tunings = tunings
rotational_PID.tunings = rotationalTunings
#leftMotor_PID.sample_time = 0.01  # update every 0.01 seconds
#rightMotor_PID.sample_time = 0.01
#rotational_PID.sample_time = 0.01
leftMotor_PID.output_limits = (-100, 100)    # output value (Duty Cycle)
rightMotor_PID.output_limits = (-100, 100)
rotational_PID.output_limits = (-100, 100)

# Odometry variables
prev_leftEncF, prev_leftEncB = 0, 0
prev_rightEncF, prev_rightEncB = 0, 0

#Rotational PID will need some odoemtry in order to work out rotation
travel, TotalTravel, thetaRad, theta = 0.0, 0.0, 0.0, 0.0
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
    if theta > 180:
        theta -= 360
    elif theta < -180:
        theta += 360
    botX += travel * math.sin(thetaRad);
    botY += travel * math.cos(thetaRad);
    print('x', int(botX), 'y', int(botY), 'theta', int(theta))

def Act(): # velocity, rotation, bearing) # Motor control PID function ----------
    # If bearing is given : do this -----
    global velocity, rotation, bearing, rotationAccuracy
    global leftDutyCycle, rightDutyCycle
    print ('Velocity', velocity, 'Rotation', round(rotation, 2))
    #bearing = theta
    bearing -= int(bearing/360) * 360
    if bearing > 180:
        bearing -= 360
    elif bearing < -180:
        bearing += 360
    rotationError = bearing - theta
    if -rotationAccuracy < rotationError < rotationAccuracy:
        rotation = 0
    else:
        rotation = rotational_PID(rotationError)
        rotation = max(min(50, rotation), -50)
    print('Rotation Error = ', round(rotationError, 2), '. Rotation = ', round(rotation, 2))
    # If bearing not involved : go straight to here ---
    leftMotor_PID.setpoint = velocity - rotation #
    rightMotor_PID.setpoint = velocity + rotation #
    print(round(leftMotor_PID.setpoint, 2), ' Setpoints ', round(rightMotor_PID.setpoint, 2))
    leftDutyCycle += leftMotor_PID(leftTicks)
    rightDutyCycle += rightMotor_PID(rightTicks)
    if leftMotor_PID.setpoint == 0:
        leftDutyCycle = 0
    if rightMotor_PID.setpoint == 0:
        rightDutyCycle = 0
    leftDutyCycle = max(min(100, leftDutyCycle), -100)  # The motor speed needs to be between 0 and 100, so clamp the value using max and min
    rightDutyCycle = max(min(100, rightDutyCycle), -100)
    print(round(leftDutyCycle, 2), ' Duty Cycles ', round(rightDutyCycle, 2))
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
            # Get State
            LB = dataList[0]
            FB = dataList[1]
            RB = dataList[2]
            # Get Action
            if LB == 0 or FB == 0 or RB == 0: # if bumpers are hit, Stop.
                if Stopped == False:
                    Stop()
                    Stopped = True
                    if LB == 0:
                        SpinLeft()
                        print("Left Hit")
                    elif FB == 0:
                        SpinRight()
                        print('Front Hit')
                    elif RB == 0:
                        SpinRight()
                        print('Right Hit')
                Act()
                    
            else:
                Stopped = False
                #Get action
                velocity = 0
                #Forward()
                Act()
            # add some data to a dictionary
            PID_data['t'].append(t)
            PID_data['A'].append(botX)
            PID_data['B'].append(botY)
            PID_data['C'].append(velocity)
            PID_data['D'].append(rotation)

            '''
            time
            Serial - info from arduino - Get State
            odometry - where t f are we - Get state
            Sensors - react to bumps etc - Vehicle 1
            Decisions/Thinking - Policy
            PID - motor control - Act
            Reward?
            '''
