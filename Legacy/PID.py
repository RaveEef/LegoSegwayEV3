#!/usr/bin/env python
import ev3dev.ev3 as ev3
import time
from datetime import timedelta


motorA = ev3.LargeMotor('outA')
motorD = ev3.LargeMotor('outD')
gyro = ev3.GyroSensor()

dt = 0.022 #time for a loop, in seconds
radius = 0.021 #radius, in metres
kp = 1
ki = 1
kd = 1
kAngle = 1
kRate = 1
kPosition = 1
kSpeed = 1
maxIndex = 7
lastIndex = 0

speedIndex = 0
speedArray = []
for i in range(1,maxIndex):
    speedArray.append(0)

dt = timedelta(seconds=(dt - 0.002)) #knock off time it takes to start the loop and for the output to get sent to motors

rateAvg = calibrate(20);
ev3.tone(375,100)
time.sleep(1)
ev3.tone(375,100)
time.sleep(1)
ev3.tone(575,100)
time.sleep(1)

while(True):
    startsAt = datetime.now()
    expiresAt = startsAt + dt

    #Get Rate
    rateReading = getGyroRate(5)
    rateAvg = rateAvg*(1-(dt.seconds*0.2)) + rateReading*(dt.seconds*0.2)
    rate = rateReading - rateAvg
    #Get Angle
    angle = angle + (dt.seconds * rate)
    #Get Speed
    motorVals = (motorA.position + motorD.position)/2.0
    speed = getMotorSpeed(motorVals)/57.3
    #Get Position
    position = motorVals/57.3
    #Get Error Signal
    #Get Output

    wait(expiresAt.seconds - startsAt.seconds)


def calibrate(loops):
    mean = 0
    for i in range(1,loops):
        mean = mean + getGyroRate(5);
    mean = mean/double(loops)
    return mean

def getGyroRate(readings):
    mean = 0
    for i in range(1,readings):
        mean = mean + gyro.rate
    mean = mean/double(readings)
    return mean

def getMotorSpeed(motorVals):
    speedIndex += 1
    if speedIndex==maxIndex:
        speedIndex = 0
    lastIndex = speedIndex+1
    if lastIndex==maxIndex:
        lastIndex = 0
    speedArray[speedIndex] = motorVals
    return (speedArray[speedIndex] - speedArray[lastIndex]) / (dt.seconds * maxIndex)
