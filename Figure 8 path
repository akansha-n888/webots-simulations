from numpy import array, empty, sign
from numpy.linalg import norm
from math import sqrt, pi, ceil, sin, cos, atan2, atan
from time import time

from PID import PIDControllers

from myRobot import *

# Set up sampling period and stopping time
T_s = 0.02
T_f = 30.0

# Create arrays for storing reference inputs
npts = ceil(T_f/T_s)+1
xRef = empty(npts)
yRef = empty(npts)
aRef = empty(npts)

# Open a file ref.csv for storing data of the reference input for plotting
h = open('ref.csv', 'w')
freq = 2.0*pi/T_f
for k in range(npts):
   t = k*T_s
   xRef[k] = 0.5 * sin(freq*t)
   yRef[k] = 0.5 * sin(2 * freq * t)
   aRef[k] = atan2(abs(yRef[k]), abs(xRef[k]))
   h.write('%10.3e %10.3e %10.3e %10.3e\n' % (t, xRef[k], yRef[k], aRef[k]))

# open a file pose.csv to store the robot pose for plotting
f = open('pose.csv', 'w')

# set up p_0
x_0 = 0.05
y_0 = -0.1
theta_0 = pi/6
p_0 = array([x_0, y_0, theta_0]).T

pRef = array([0.0, 0.0, 0.0]).T
pd_dot = array([0.0, 0.0, 0.0]).T
w = array([0.0, 0.0, 0.0]).T

# Initialize PID controller
Kp = array([5.0, 10.0, 5.0]).T
Ki = array([10.0, 20.0, 10.0]).T
Kd = array([0.01, 0.01, 0.01]).T
pid = PIDControllers(Kp, Ki, Kd, T_s)

# Initialize robot and other variables

robot = myRobot(T_s)
robot.initialize(theta_0)

current_time = 0.0

p = p_0

# Set start time to current time
start_time = time()

# Control Loop
for k in range(npts):
   robot.get_readings_update()

   # write data to file
   f.write('%10.3e %10.3e %10.3e %10.3e\n' % (current_time, p[0], p[1], p[2]))

   # get current robot orientation
   theta = p[2]

   # Determine p_dot using forward kinematics
   p_dot = robot.forward_kinematics(robot.angular_velocities, theta)

   # Determine value of reference path p_r at current time
   pRef = array([xRef[k], yRef[k], aRef[k]]).T

   # Determine pd_dot (desired p_dot) using PID control law
   pd_dot = pid(pRef, p)

   # Determine desired wheel velocities using inverse kinematics
   w = robot.inverse_kinematics(pd_dot, theta)

   # Apply motor limits
   lim = 5*pi
   w = robot.motor_limit(w, lim)

   # Execute the motion
   robot.set_angular_velocities(w)

   # odometry update
   p = p + p_dot*T_s

   # replace calculated update for theta with measured value from IMU
   p[2] = robot.orientation

   # time update
   current_time = time() - start_time

robot.stop()
robot.close()

# Close all files
f.close()
