import numpy as np
import math
import time
from numpy import array, dot

from myRobot import myRobot

theta0 = 30.0*math.pi/180.0

run_time = 2.5
Ts = 0.02

robot = myRobot(Ts)

robot.initialize(theta0)

print('Moving forward')

elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   robot.get_readings_update()
   theta = robot.orientation
   robot.move_forward(0.2, theta)
   elapsed_time = time.time() - t0

robot.stop()

print('Rotating CW')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   robot.get_readings_update()
   theta = robot.orientation
   robot.rotate_CW(0.75, theta)
   elapsed_time = time.time() - t0

robot.stop()

print('Moving right')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   robot.get_readings_update()
   theta = robot.orientation
   robot.move_right(0.2, theta)
   elapsed_time = time.time() - t0

robot.stop()

print('Rotating CCW')
elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   robot.get_readings_update()
   theta = robot.orientation
   robot.rotate_CCW(0.75, theta)
   elapsed_time = time.time() - t0

robot.stop()


print('Moving forward')

elapsed_time = 0.0
t0 = time.time()
while (elapsed_time < run_time):
   robot.get_readings_update()
   theta = robot.orientation
   robot.move_forward(0.2, theta)
   elapsed_time = time.time() - t0

robot.stop()

robot.close()