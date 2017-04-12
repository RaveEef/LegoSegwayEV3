#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from ev3dev import *
# import time
from datetime import timedelta
import datetime


def getGyroRate(readings):
    mean = 0
    for i in range(1, readings):
        mean = mean + gyro.rate
    mean = mean / readings
    return mean


def calibrate_gyro(loops):
    mean = 0
    for i in range(1, loops):
        mean = mean + getGyroRate(5)
    mean = mean / loops
    return mean


# def calibrate_position(nb_loops, motor_l, motor_r):
#     mean_l = mean_r = 0
#     for i in range(nb_loops):
#         mean_l += motor_l.position_sp
#         mean_r += motor_r.position_sp
#     mean_r = mean_r / nb_loops
#     mean_l = mean_l / nb_loops
#     return mean_l, mean_r


# Constants
DT = timedelta(seconds=(0.022 - 0.002))  # Loop time (s), inc. overhead
RADIUS = 0.021  # Radius (metres)
SPEED_ARR_LENGTH = 7
MAX_ABS_PWM = 100

# Data arrays
speed_arr = [0] * SPEED_ARR_LENGTH

# PID Parameters
kp = 1
ki = 0
kd = 0
k_angle = 1
k_rate = 1
k_pos = 1
k_speed = 1
max_abs_control_val = 1000

# Motors and Sensors
# Left and right defined from robot viewpoint with screen as face
motor_l = ev3.LargeMotor('outD')
motor_r = ev3.LargeMotor('outA')
gyro = ev3.GyroSensor()

# # Unnecessary noises..
# motor_l, motor_r = calibrate_position(20, motor_l, motor_r)
# ev3.Sound.tone(375,100)
# time.sleep(1) #Lol
# ev3.Sound.tone(375,100)
# time.sleep(1)
# ev3.Sound.tone(575,100)
# time.sleep(1)

# Calibrate
gyro_rate_mean = calibrate_gyro(20)
gyro_central = gyro.angle

# Set motor position to zero
# Allow on-the-fly speed updates using duty_cycle_sp
motor_l.command('reset')
motor_l.command('run-direct')
motor_r.command('reset')
motor_r.command('run-direct')

prev_error = 0
sum_error = 0
while(True):
    startsAt = datetime.datetime.now()  # TODO: Why this over a timer?
    expiresAt = startsAt + DT

    # rateReading = getGyroRate(5)
    # rateAvg = rateAvg*(1-(DT.seconds*0.2)) + rateReading*(DT.seconds*0.2)
    # rate = rateReading - rateAvg

    angle = gyro.angle + (DT.seconds * rate)
    rate = getGyroRate(5)
    position = (motor_l.position + motor_r.position) / 2
    # position = ((motor_l.position + motor_r.position)/2.0)/57.3
    speed = (motor_l.speed + motor_r.speed) / 2

    # PID control
    control_val = 0

    # Rate, pos and speed all have set point 0
    cur_error = ((k_angle * (angle - gyro_central)) +
                 (k_rate * rate) +
                 (k_pos * position) +
                 (k_speed * speed))
    sum_error += cur_error

    # TODO integral windup gaurd?

    control_val += kp * cur_error  # Proportional term
    control_val += ki * sum_error  # Integral term
    control_val += kd * (cur_error - prev_error)  # Derivative term

    # TODO could be removed if precision works out for control values as
    # [-100, 100]
    control_val = control_val * (MAX_ABS_PWM / max_abs_control_val)

    # Clip control value to +/- MAX_ABS_PWM
    new_speed = max(-MAX_ABS_PWM, min(control_val, MAX_ABS_PWM))

    motor_l.duty_cycle_sp = new_speed
    motor_r.duty_cycle_sp = new_speed

    prev_error = cur_error

    # Wait until end of minimum control update period
    while(datetime.datetime.now() < expiresAt):
        pass
