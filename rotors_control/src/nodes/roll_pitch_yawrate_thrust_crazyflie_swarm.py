#!/usr/bin/env python2
"""Class for roll pitch yawrate thrust."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
# from ros_interface import ROSControllerNode

# Import classes
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist
#from nav_msgs.msg import Odometry

# NOTES:
# self.func_name() is it correct?
# self.thrust_percentage value
# self.thrust --> is it command vs. actual

class roll_pitch_yawrate_thrust_crazyflie(object):
    """roll, pitch, yawrate, and thrust for crazyflie."""

    # write code here for roll pitch yawrate thrust

    def __init__(self, pitch_c, roll_c, yaw_rate_c, p, q, r, roll, pitch, yaw, thrust_c):

        self.M_PI = 3.14159265358979323846 # pi
        self.ANGULAR_MOTOR_COEFFICIENT = 1.8#1.8#0.2685 # ANGULAR_MOTOR_COEFFICIENT
        self.MOTORS_INTERCEPT = 2300#2270#426.24 #2270 # # MOTORS_INTERCEPT [rad/s]
        self.SAMPLING_TIME = 1.0/100 # SAMPLING TIME [s] change?
        self.MAX_PROPELLERS_ANGULAR_VELOCITY = 2618#2618 # MAX PROPELLERS ANGULAR VELOCITY [rad/s]

        # Roll pitch yawrate thrust controller parameters
        # Vals can be found at: src/CrazyS/rotors_gazebo/resource/roll_pitch_yawrate_thrust_controller_crazyflie2.yaml
        self.attitude_gain_kp = [0.0611, 0.0611]
        self.attitude_gain_ki = [0.0349, 0.0349]
        self.rate_gain_kp = [1000,1000,1000]#[1000, 1000, 1000]
        self.rate_gain_ki = [0.0, 0.0, 95.6939] #95.6839
        self.thrust_percentage = 1.0

        # commands
        self.pitch_c = pitch_c#np.degrees(pitch_c)
        self.roll_c = roll_c#np.degrees(roll_c)
        self.yaw_rate_c = yaw_rate_c#np.degrees(yaw_rate_c)
        self.thrust = thrust_c
#        print("pitch_c: ", self.pitch_c, "roll_c: ", self.roll_c)
#        print("yaw_rate_c: ", self.yaw_rate_c, "thrust_c: ", self.thrust)

        # current state
        self.p = p #np.degrees(p)
        self.q = q #np.degrees(q)
        self.r = r #np.degrees(r)
        self.roll = roll#np.degrees(roll)
        self.pitch = pitch #np.degrees(pitch)
        self.yaw = yaw #np.degrees(yaw)
#        print("p: ", self.p, "q: ", self.q, "r: ", self.r)
#        print("roll: ", self.roll, "pitch: ", self.pitch, "yaw: ", self.yaw)
         
        # initialize variables
        self.p_command_ki_ = 0 # change? i have no idea if this is correct
        self.q_command_ki_ = 0
        self.delta_psi_ki_ = 0


    def CalculateRotorVelocities(self):
    
        PWMs = self.ControlMixer() #change?
        PWM_1 = PWMs[0]
        PWM_2 = PWMs[1]
        PWM_3 = PWMs[2]
        PWM_4 = PWMs[3]

        print("PWMs: ", PWMs)

        omega_1 = ((PWM_1 * self.ANGULAR_MOTOR_COEFFICIENT) + self.MOTORS_INTERCEPT)
        omega_2 = ((PWM_2 * self.ANGULAR_MOTOR_COEFFICIENT) + self.MOTORS_INTERCEPT)
        omega_3 = ((PWM_3 * self.ANGULAR_MOTOR_COEFFICIENT) + self.MOTORS_INTERCEPT)
        omega_4 = ((PWM_4 * self.ANGULAR_MOTOR_COEFFICIENT) + self.MOTORS_INTERCEPT)

        print("omegas: ",[omega_1,omega_2,omega_3,omega_4])

        # The omega values are saturated considering physical constraints of the system
        if(not (omega_1 < self.MAX_PROPELLERS_ANGULAR_VELOCITY and omega_1 > 0)):
            if(omega_1 > self.MAX_PROPELLERS_ANGULAR_VELOCITY):
                omega_1 = self.MAX_PROPELLERS_ANGULAR_VELOCITY
            else:
                omega_1 = 0

        if(not (omega_2 < self.MAX_PROPELLERS_ANGULAR_VELOCITY and omega_2 > 0)):
            if(omega_2 > self.MAX_PROPELLERS_ANGULAR_VELOCITY):
                omega_2 = self.MAX_PROPELLERS_ANGULAR_VELOCITY
            else:
                omega_2 = 0

        if(not (omega_3 < self.MAX_PROPELLERS_ANGULAR_VELOCITY and omega_3 > 0)):
            if(omega_3 > self.MAX_PROPELLERS_ANGULAR_VELOCITY):
                omega_3 = self.MAX_PROPELLERS_ANGULAR_VELOCITY
            else:
                omega_3 = 0

        if(not (omega_4 < self.MAX_PROPELLERS_ANGULAR_VELOCITY and omega_4 > 0)):
            if(omega_4 > self.MAX_PROPELLERS_ANGULAR_VELOCITY):
                omega_4 = self.MAX_PROPELLERS_ANGULAR_VELOCITY
            else:
                omega_4 = 0

        rotor_velocities = [omega_1, omega_2, omega_3, omega_4]
        return rotor_velocities

    def ControlMixer(self):
        self.thrust = self.thrust * self.thrust_percentage 

        delta_vars = self.RateController() # change?
        delta_phi = delta_vars[0]
        delta_theta = delta_vars[1]
        delta_psi = delta_vars[2]
        print('delta_vars: ', delta_vars)

        PWM_1 = self.thrust - (delta_theta/2) - (delta_phi/2) - delta_psi
        PWM_2 = self.thrust + (delta_theta/2) - (delta_phi/2) + delta_psi
        PWM_3 = self.thrust + (delta_theta/2) + (delta_phi/2) - delta_psi
        PWM_4 = self.thrust - (delta_theta/2) + (delta_phi/2) + delta_psi

        PWMs = [PWM_1, PWM_2, PWM_3, PWM_4]
        return PWMs

    def RateController(self):

        #command p, q, r (from attitude and position controller)  
#        pq_commands = self.AttitudeController() # self prescript??
#        p_command = pq_commands[0]
#        q_command = pq_commands[1]
        p_command = self.roll_c
        q_command = self.pitch_c
        r_command = self.yaw_rate_c

        # compute errors
        p_error = p_command - self.p
        q_error = q_command - self.q
        r_error = r_command - self.r

        # delta phi and delta theta calculation
        delta_phi = self.rate_gain_kp[0] * p_error
        delta_theta = self.rate_gain_kp[1] * q_error

        # delta psi calculation
        delta_psi_kp = self.rate_gain_kp[2] * r_error
        self.delta_psi_ki_ = self.delta_psi_ki_ + (self.rate_gain_ki[2] * r_error * self.SAMPLING_TIME)
        delta_psi = delta_psi_kp + self.delta_psi_ki_

        # return
        delta_vars = [delta_phi, delta_theta, delta_psi]
        return delta_vars

#    def AttitudeController(self):

#        theta_command = self.pitch_c
#        phi_command = self.roll_c

#        phi_error = phi_command - self.roll
#        theta_error = theta_command - self.pitch

#        p_command_kp = self.attitude_gain_kp[0] * phi_error
#        self.p_command_ki_ = self.p_command_ki_ + (self.attitude_gain_ki[0] * phi_error * self.SAMPLING_TIME)
#        p_command = p_command_kp + self.p_command_ki_

#        q_command_kp = self.attitude_gain_kp[1] * theta_error
#        self.q_command_ki_ = self.q_command_ki_ + (self.attitude_gain_ki[1] * theta_error * self.SAMPLING_TIME)
#        q_command = q_command_kp + self.q_command_ki_

#        pq_commands = [p_command, q_command]
#        return pq_commands
