from math import sin, cos, exp, sqrt, pi
import numpy
from numpy import array, dot

from Robot3WD import Robot

class myRobot(Robot):
    def __init__(self, sampling_period, wheel_radius=None, L=None):
        Robot.__init__(self, sampling_period, wheel_radius, L)

    # --------------------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 2                                                         #
    # --------------------------------------------------------------------------------------#
    def inverse_kinematics(self, p_dot, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        
        M_theta = numpy.dot(1 / wheel_radius, array([[sin(theta), -cos(theta), -L],
                                            [cos(pi / 6 + theta), sin(pi / 6 + theta), -L],
                                            [-cos(pi / 6 - theta), sin(pi / 6 - theta), -L]]))
        wheel_angular_velocities = numpy.dot(M_theta,p_dot)
        return wheel_angular_velocities

    def move_left(self, vx, theta):
        p_dot = array([-vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_forward(self, vy, theta):
        p_dot = array([0.0, vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_backward(self, vy, theta):
        p_dot = array([0.0, -vy, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def move_right(self, vx, theta):
        p_dot = array([vx, 0.0, 0.0]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def rotate_CCW(self, w, theta):
        p_dot = array([0.0, 0.0, w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)

    def rotate_CW(self, w, theta):
        p_dot = array([0.0, 0.0, -w]).T
        w = self.inverse_kinematics(p_dot, theta)
        self.set_angular_velocities(w)


    # --------------------------------------------------------------------------------------#
    # Pre-Lab work for Experiment 3                                                         #
    # --------------------------------------------------------------------------------------#
    def forward_kinematics(self, wheel_angular_velocities, theta):
        L = self._L
        wheel_radius = self._wheel_radius
        M = numpy.dot(wheel_radius / 3, array([[2*sin(theta), 2*cos(theta+pi/6), -2*sin(theta+pi/3)],
                                            [-2*cos(theta), 2*cos(theta-pi/3), 2*cos(theta+pi/3)],
                                            [-1/L, -1/L, -1/L]]))
        p_dot = numpy.dot(M,wheel_angular_velocities)
        return p_dot

    def motor_limit(self, wheel_angular_velocities, limit_value):
        wheel_angular_velocities_bar = array([0.0, 0.0, 0.0]).T
    
        for i in range(3):
            if wheel_angular_velocities[i] < -limit_value:
                wheel_angular_velocities_bar[i] = -limit_value
            elif wheel_angular_velocities[i] > limit_value:
                wheel_angular_velocities_bar[i] = limit_value
            else:
                wheel_angular_velocities_bar[i] = wheel_angular_velocities[i]
        return wheel_angular_velocities_bar

#### end of myrobot Class ###

# --------------------------------------------------------------------------------------#
# Pre-Lab work for Experiment 4                                                         #
# --------------------------------------------------------------------------------------#

# Define the HMatrix function
    def HMatrix(self,q):
       h11 = cos(q[2]) 
       h12 = -sin(q[2])
       h13 = q[0] 
       
       h21 = sin(q[2])
       h22 = cos(q[2])
       h23 = q[1] 
       
       h31 = 0
       h32 = 0
       h33 = 1.0 
       
       H = array([[h11, h12, h13], [h21, h22, h23], [h31, h32, h33]])
       return H

# Define the Vraw_to_distance function
    def Vraw_to_distance(self,Vraw):
   #
   # ... (Fill in rest of code below) ...
   #
       c1 = 0.6209
       c2 = 0.0541
       d = c1*exp(-c2*sqrt(Vraw))
       return d

#### end of myrobot.py ###
