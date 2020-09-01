'''
Program for Terence with obstacle avoidance on which to build occupancy grid and SLAM experiments.
28 07 20 created basic turn on the spot bumper program.
17 08 20 adding gubbins for Q Tables to dictate reactions to sonar stimulus (obstacle avoidance).
To Do -
need a way to stop QLearn adjusting q table for the last of a short state with the first subsequent long state and visa versa.
bool shortLast longLast
Braitenberg experiment with eight sonar
integrate odometry with rotational PID
Build occupancy grid and map environment.
'''
# Import Stuff
import RPi.GPIO as GPIO
import time
import serial
import math
import random
import numpy as np
from numpy import save
from numpy import load
import os.path
import matplotlib.pyplot as plt
import pickle

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

SQ = load('Q_Table_QL_256_Explore.npy') # Load short range 256 state Q Table
LQ = load('Q_Table_QL_16_Explore.npy') # Load long range 16 state Q table
shortStates = load('States_List_QL_256_Explore.npy') # Load short 256 state list
longStates = load('States_List_QL_16_Explore.npy') # Load long 16 state list

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
            # 3 = Left Sonar No 1, 4 = Left Secondary Sonar No 2, 5 = Front Left Secondary Sonar No 3, 6 = Front Left Sonar No 4
            # 7 = Front Right Sonar No 5, 8 = Front Right Secondary Sonar No 6, 9= Right Secondary Sonar No 7, 10 = Right Sonar No 8.
            # 11 = left forward 12 = left back 13 = right forward 14 = right back encoders
    except:
        print('no Connection')
        
def SONAR(position): # Retrieve the state of individual Sonar
    global dataList
    # Eight Sonar Script
    if position == 0:
        distance = dataList[3]
        
    if position == 1:
        distance = dataList[4]
        
    if position == 2:
        distance = dataList[5]
        
    if position == 3:
        distance = dataList[6]
        
    if position == 4:
        distance = dataList[7]
        
    if position == 5:
        distance = dataList[8]
        
    if position == 6:
        distance = dataList[9]
        
    if position == 7:
        distance = dataList[10]
        
    return int(distance)
         
SHORT_DISTANCE = 9 # threshold in cm over which obstacles are ignored
LONG_DISTANCE = 27 # threshold in cm over which obstacles are ignored
short, long = False, False
crashed = False
S = np.zeros(8)
ss, ls, lastss, lastls = 0, 0, 0, 0
shortState = np.zeros(8)
longState = np.zeros(4)
    
def getState(): # Returns state of the percieved world as a list i,e, distances from sonars and speed of wheels
    global shortStates, longStates, S, ss, ls, lastss, lastls, shortState, longState, SHORT_DISTANCE, LONG_DISTANCE, short, long
    S[0] = SONAR(0)  # read left sonar and get distance value
    S[1] = SONAR(1)  # 
    S[2] = SONAR(2)  # 
    S[3] = SONAR(3)  # read left front sonar
    S[4] = SONAR(4)  # read right front sonar
    S[5] = SONAR(5)  # 
    S[6] = SONAR(6)  # 
    S[7] = SONAR(7)  # Read right most sonar
    
    # Convert sonar distance values into boolean for state lists
    for x in range(0, 8):
        if S[x] > SHORT_DISTANCE or S[x] < 1: # newPing returns distances over 100cm as 0
            shortState[x] = 0
        else: # if S[x] < SHORT_DISTANCE + 1 and S[x] > 0:
            shortState[x] = 1
            
    for y in range (0, 4):
        num = y*2
        if S[num] < LONG_DISTANCE and S[num] > 0 or S[num+1] < LONG_DISTANCE and S[num+1] > 0:
            longState[y] = 1
        else:
            longState[y] = 0
    print ('short state ', shortState)
    #print ('long state ', longState)
    lastss = ss
    lastls = ls
    ss = np.argwhere((shortStates == shortState).all(axis=1))#
    ls = np.argwhere((longStates == longState).all(axis=1))#
    #print('ss and ls', ss, ',', ls)
    #
    if ss > 0:
        if short == False:
            short = True
            startT = t + 1
    elif ls > 0:
        if long == False:
            long = True
            startT = t + 1
    if ss == 0:
        short = False
    if ls == 0:
        long = False
    #print ("New State = ", newState)
    #print ("State, s = ", s)
    return ss, ls, lastss, lastls                   # return list index to retieve data about state and action values (Q values)

REWARD_LIST = np.array([-1, -2, -3, -4, -4, -3, -2, -1])

def getReward():
    global dataList, crashed, shortStates, ss, short, long
    r = 0
    if short == True: 
        reward = np.multiply(shortStates[ss], REWARD_LIST)
        print('State Reward = ', reward)
        r += np.sum(reward)
    if crashed == True:
        r -= 100
    #print ("Reward = ", r)
    r = round(r, 2)
    return (r)

def QLearn():
    global SQ, LQ, ss, ls, lastss, lastls, shortStates, longStates, alpha, gamma, a
    #print('a ', a)
    if short == True:
        newS = ss
        Q = SQ
        currentQ = Q[lastss, a] # q[s,a] this is the value which needs updating based on the new staste index ss
    elif long == True:
        newS = ls
        Q = LQ
        currentQ = Q[lastls, a]
    r = getReward() # get reward based on action and distance from obstacles
    #print ("Old State = ", states[s])
    #newS = getState() # newS has already been gotten outside the QLearn loop.
    max_future_Q = np.max(Q[newS]) # Get Q Value of optimum action.
    #print ("currentQ = ", currentQ)
    #newQ = (1 - alpha) * currentQ + alpha * (r + gamma * max_future_Q)  # got from https://pythonprogramming.net/q-learning-reinforcement-learning-python-tutorial/
    newQ = currentQ + alpha * (r + gamma * max_future_Q - currentQ) # Bellman equation, the heart of the Reinforcement Learning program
    newQ = np.round(newQ, 2) # round down floats for a tidier Q Table
    currentQ = newQ
    #s = newS
    #print ("NewQ = ", newQ)

# Number of Actions = 6 # drive forward at 50%, spin left, spin right, turn Left, turn Right or reverse.
def getAction(): # pass the s index of Q table and epsilon, to get maxQ make epsilon 1
    global SQ, LQ, ss, ls, epsilon
    randVal = 0
    #Epsilon Greedy - 
    randVal = random.randrange(1,101)
    if randVal <= (1-epsilon)*100:
        if  short == True:
            action = np.argmax(SQ[ss]) # moves 1, 2 and 3
        elif long == True:
            action = np.argmax(LQ[ls]) # moves 0, 2 and 4
    else:
        action = random.randrange(0,3)
        #print("Random Action = ", action, ", Random Value = ", randVal, ", Epsilon = ", epsilon)
    #print('Action ', action)
    return(action)  

def Act(action):
    global short, long, leftDutyCycle, rightDutyCycle
    if  short == True:
        action += 1 # 0, 1, 2 become 1, 2, 3
        leftDutyCycle, rightDutyCycle = 50, 50
    elif long == True:
        action *= 2 # 0, 1, 2 become 0, 2, 4.
        leftDutyCycle, rightDutyCycle = 75, 75
        
    if action == 0: # Turn Left
        TurnLeft()
    if action == 1: # Spin Left
        SpinLeft()
    if action == 2: # Drive Forward
        Forward()
    if action == 3: # Spin Right
        SpinRight()
    if action == 4: # Turn Right
        TurnRight()
    if action == 5: # Reverse
        Reverse()
        
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

t = 0
    
epsilon = 0.3
EPSILON_END = 1
EPSILON_DECAY = epsilon/EPSILON_END
if 0 <= t < EPSILON_END:
    epsilon -= EPSILON_DECAY * t
else:
    epsilon = 0

#Computational parameters
alpha = 0.01    #"Forgetfulness" weight or learning rate.  The closer this is to 1 the more weight is given to recent samples.
gamma = 0.99   #look-ahead weight or discount factor 0 considers new rewards only, 1 looks for long term rewards

# Step/time parameters
lasttime = time.time() # Variable to store time for timesteps
step = 0
previousStep = 0

startT = t + 1 # Set starting itteration number based on info saved in log

print("Start")
print("setting up Serial")
time.sleep(1)
print("Getting Data")
while noData == True:
    Serial()
print("DATA", dataList)
# getState() # s = index of state in states list

while True:

    Pause()
    if pause == 1:
        pass
    else:
        if  time.time() > lasttime + 0.1: # 0.05 = 75 millis = 13.3 Hertz - 50 milliseconds = 20 Hertz
            lasttime = time.time()
            step = time.time() - previousStep
            previousStep = time.time()
            while noData == True:
                Stop()
                Serial()
            Serial()
            LB = dataList[0]
            FB = dataList[1]
            RB = dataList[2]
            if LB == 0 or FB == 0 or RB == 0: # if bumpers are hit, Stop.
                if Stopped == False:
                    Stop()
                    Stopped = True
                    crashed = True
                    if short or long:
                        QLearn()
                    #React to obstruction
                    leftDutyCycle, rightDutyCycle = 50, 50
                    if LB == 0:
                        SpinLeft()
                        print("Left Hit")
                    elif FB == 0:
                        Reverse()
                        print('Front Hit')
                    elif RB == 0:
                        SpinRight()
                        print('Right Hit')
                    time.sleep(.1)
                    getState()
                    startT = t + 1
                    Stopped = False
                    crashed = False
                    
            else:           # Get State and if all is well, states < 1, act freely otherwise run QLearning loop
                getState()
                if short == True:
                    #print ('SHOOOOORT')
                    pass
                if long == True:
                    #print ('LOOOOOOOONG')
                    pass
                if short == False and long == False:
                    #print('FREEEEEEEEEEEEEEEEEEEEEEEEE')
                    leftDutyCycle, rightDutyCycle = 100, 100
                    a = 2
                    # do what you like - get action?
                else:       # QLearning loop if states are > 0
                    t += 1
                    if epsilon > 0:
                        epsilon -= EPSILON_DECAY # reduces to 0 over 10,000 steps
                    #print("Iteration = ", t)
                    if t > startT: # on the first time through the loop there will be no reward or previous states or actions
                        QLearn()
                    a = getAction()   # getAction will find an action based on the index s in the Q list and exploration will be based on epsilon
                #print("Action = ", a)
                Act(a)
