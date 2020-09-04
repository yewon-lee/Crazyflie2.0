#!/usr/bin/env python2

"""
ROS interface for controlling up to four Cf2.0's and running the flocking algorithm.

This ROS node subscribes to the following topics:
/crazyflie2_id/odometry

This ROS node publishes to the following topics:
/crazyflie2_id/command/motor_speed
/crazyflie2_id/goal

Where id is a number from 0 to 4

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
import math
import time
import random

import csv
import sys
import os

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from position_controller import PositionController
from std_msgs.msg import Empty, String

# Import classes - addendum

#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # subscribe
from geometry_msgs.msg import PoseStamped # publish
from nav_msgs.msg import Odometry # subscribe
from mav_msgs.msg import Actuators # publish
from roll_pitch_yawrate_thrust_crazyflie import roll_pitch_yawrate_thrust_crazyflie # change?


class position_controller_flock_node(object):
    """ROS interface for controlling up to four Cf2.0's in Gazebo and running flocking algorithm."""


    def __init__(self, uav_ids, init, fin, vx_ds, vy_ds, vz_ds, yaw_ds):

        self.cf_ids = uav_ids
        self.number_of_agents = np.shape(uav_ids)[0]
        self.takoff_alt = 1.0 # change?
        self._pos = {}
        self._vel = {}
        self._quat = {}
        self._dist_to_goal = {}

        self._euler_angles = {}
        self._euler_angular_rates = {}
        self._z_old = {
            '0' : 0.0,
            '1' : 0.0,
            '2' : 0.0,
            '3' : 0.0
        }
        self._z_oold = {
            '0' : 0.0,
            '1' : 0.0,
            '2' : 0.0,
            '3' : 0.0
        }
        self.yaw_old = {
            '0' : 0.0,
            '1' : 0.0,
            '2' : 0.0,
            '3' : 0.0
        }

        self.radius = 0.15
        self.d_star = self.radius
        self.MaxVelo = 1.0
        # Tune these
        self.c1 = 7*4.5
        self.c2 = 9*4.5
        self.RepulsiveGradient = 7.5*100
        
        self.previous_time = 0.0
        self.change_time = -100.0

        self.controller = PositionController()
        self.init_time = rospy.get_time()
        

        ### Publish ###
        # waypoint messages
        self.goal_msg_0, self.goal_msg_1, self.goal_msg_2, self.goal_msg_3 = PoseStamped(), PoseStamped(), PoseStamped(), PoseStamped()
        self.goal_msgs = {
            '0' : self.goal_msg_0,
            '1' : self.goal_msg_1,
            '2' : self.goal_msg_2,
            '3' : self.goal_msg_3
        }        

        # rotor velocities messages
    #    self.cmdV_msg_0, self.cmdV_msg_1, self.cmdV_msg_2, self.cmdV_msg_3 = Actuators(), Actuators(), Actuators(), Actuators()
    #    self.rotor_vel_msgs = {
    #        '0' : self.cmdV_msg_0,
    #        '1' : self.cmdV_msg_1,
    #        '2' : self.cmdV_msg_2,
    #        '3' : self.cmdV_msg_3
    #    }   

        ### Subscribe ###
        # odometry messages
        self._currpos_callbacks = {
            '0' : self._currpos_callback_0,
            '1' : self._currpos_callback_1,
            '2' : self._currpos_callback_2,
            '3' : self._currpos_callback_3
        }

        # set parameters
        self.initials, self.finals = {}, {}
        self.goal_pubs, self.cmdVtemp_pubs, self.cmdV_pubs, self.takeoffs, self.lands = {}, {}, {}, {}, {}
        for index, cf_id in enumerate(self.cf_ids):
            self.initials[str(cf_id)] = init[index][:]
            self.finals[str(cf_id)] = fin[index][:]
            self.goal_pubs[str(cf_id)] = rospy.Publisher('/crazyflie2_' + str(cf_id) +'/goal', PoseStamped, queue_size=1)
            self.cmdV_pubs[str(cf_id)] = rospy.Publisher('/crazyflie2_' + str(cf_id) +'/command/motor_speed', Actuators, queue_size=1)
            rospy.Subscriber("/crazyflie2_" + str(cf_id) + "/odometry", Odometry, self._currpos_callbacks[str(cf_id)]) 
        self.vx_d = vx_ds
        self.vy_d = vy_ds
        self.vz_d = vz_ds
        self.yaw_d = yaw_ds


        self.takeoffed = False
        self.reached_1st = False

        self.flag = {
            'flying' : 0,
            'landed' : 0,
            'preland': 0
        }

#        self.vstate = {
#            'takeoff' : self.do_takeoff,
#            'wpnav' : self.do_wpnav,
#            'land' : self.do_land
#        }

        # Position controller
        self.controller = PositionController()


    def set_rotor_vel(self, pitch_c, roll_c, yaw_rate_c, p, q, r, roll, pitch, yaw, thrust): # change arg names?

        # get conversions
        rotorvel_converter = roll_pitch_yawrate_thrust_crazyflie(pitch_c, roll_c, yaw_rate_c, p, q, r, roll, pitch, yaw, thrust)
        rotor_velocities = rotorvel_converter.CalculateRotorVelocities()

        # publish rotor velocities to Actuator
        msg = Actuators()
        msg.angular_velocities = rotor_velocities 
        msg.header.stamp = self._currpos_msg.header.stamp
        self.pub_rotor_vel.publish(msg)

    def _currpos_callback(self, msg):
        self._currpos_msg = msg
    
    def get_data(self,id):
        print(self._z_old[str(id)])
        self._z_oold[str(id)] = self._z_old[str(id)]
        self._z_old[str(id)] = self._pos[str(id)][2] # current z
        self.yaw_old[str(id)] = self._euler_angles[str(id)][2] # current yaw

    def _currpos_callback_0(self, data):
        self._pos['0'] = np.asarray(data.pose.pose.position)
        self._vel['0'] = np.asarray(data.twist.twist.linear)
        self._quat['0'] = np.array([data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y, 
                                    data.pose.pose.orientation.z, 
                                    data.pose.pose.orientation.w]) # gives quaternion object
        self._euler_angles['0'] = euler_from_quaternion(self._quat['0']) # gives roll, pitch, yaw
        self._euler_angular_rates['0'] = np.asarray(data.twist.twist.angular) # gives p, q, r

    def _currpos_callback_1(self, data):
        self._pos['1'] = np.asarray(data.pose.pose.position)
        self._vel['1'] = np.asarray(data.twist.twist.linear)
        self._quat['1'] = np.array([data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y, 
                                    data.pose.pose.orientation.z, 
                                    data.pose.pose.orientation.w]) # gives quaternion object
       # print(self._quat['1'], type(self._quat['1']))
        self._euler_angles['1'] = euler_from_quaternion(self._quat['1']) # gives roll, pitch, yaw
        self._euler_angular_rates['1'] = np.asarray(data.twist.twist.angular) # gives p, q, r

    def _currpos_callback_2(self, data):
        self._pos['2'] = np.asarray(data.pose.pose.position)
        self._vel['2'] = np.asarray(data.twist.twist.linear)
        self._quat['2'] = np.array([data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y, 
                                    data.pose.pose.orientation.z, 
                                    data.pose.pose.orientation.w]) # gives quaternion object
        self._euler_angles['2'] = euler_from_quaternion(self._quat['2']) # gives roll, pitch, yaw
        self._euler_angular_rates['2'] = np.asarray(data.twist.twist.angular) # gives p, q, r

    def _currpos_callback_3(self, data):
        self._pos['3'] = np.asarray(data.pose.pose.position)
        self._vel['3'] = np.asarray(data.twist.twist.linear)
        self._quat['3'] = np.array([data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y, 
                                    data.pose.pose.orientation.z, 
                                    data.pose.pose.orientation.w]) # gives quaternion object
        self._euler_angles['3'] = euler_from_quaternion(self._quat['3']) # gives roll, pitch, yaw
        self._euler_angular_rates['3'] = np.asarray(data.twist.twist.angular) # gives p, q, r


    def update_pos(self, id, pos):
        self.goal_msgs[id].header.seq += 1
        self.goal_msgs[id].header.frame_id = '/world' # change?
        self.goal_msgs[id].header.stamp = rospy.Time.now()

        self.goal_msgs[id].pose.position.x = pos[0]
        self.goal_msgs[id].pose.position.y = pos[1]
        self.goal_msgs[id].pose.position.z = pos[2]

        self.goal_msgs[id].pose.orientation.x = 0
        self.goal_msgs[id].pose.orientation.y = 0 
        self.goal_msgs[id].pose.orientation.z = 0
        self.goal_msgs[id].pose.orientation.w = 1
    
    def update_rotor_vels(self, id):
    # this function takes in the UAV's id and computes+publishes the rotor velocities to that UAV

        # get z_oold, z_old, and desired position/yaw
        self.get_data(id)                 

        # get roll/pitch/yawrate/thrust commands from position controller
        roll_c, pitch_c, z_dot_c, yaw_dot_c = self.controller.get_command(
            self._pos[str(id)][0], self._pos[str(id)][1], self._pos[str(id)][2], # change? x,y,z
            self._euler_angles[str(id)][0], self._euler_angles[str(id)][1], self._euler_angles[str(id)][0][2], # change? roll, pitch, yaw
            self.initials[str(id)][0], self.initials[str(id)][1], self.initials[str(id)][2], # change? xd, yd, zd
            self.vx_d[int(id)], self.vy_d[int(id)], self.vz_d[int(id)], self.yaw_d[int(id)])

        # obtain p,q,r/roll,pitch,yaw for UAV id from odometry subscription
        p = self._euler_angular_rates[str(id)][0]
        q = self._euler_angular_rates[str(id)][1]
        r = self._euler_angular_rates[str(id)][2]
        roll = self._euler_angles[str(id)][0]
        pitch = self._euler_angles[str(id)][1]
        yaw = self._euler_angles[str(id)][2]

        # convert above commands to rotor velocity commands
        rotorvel_converter = roll_pitch_yawrate_thrust_crazyflie(pitch_c, roll_c, yaw_dot_c, p, q, r, roll, pitch, yaw, z_dot_c)
        rotor_velocities = rotorvel_converter.CalculateRotorVelocities() # this yields a 4-element list

        # publish rotor velocities to Actuator
        rotorvel_msg = Actuators()
        rotorvel_msg.angular_velocities = rotor_velocities
        #rotorvel_msg.header.stamp = self._currpos_msg.header.stamp
        self.cmdV_pubs[str(cf_id)].publish(rotorvel_msg)

    def publish_msg(self):
        for cf_id in self.cf_ids:
            self.goal_pubs[str(cf_id)].publish(self.goal_msgs[str(cf_id)])

    def do_wpnav(self):
        print('Navigating!!')
        # Check all cfs reached assigened takeoff alt
        if not self.takeoffed:
            count = 0
            for cf_id in self.cf_ids:
                if abs(self._pos[str(cf_id)][2] - self.takoff_alt) < 0.05:
                    count += 1
                position_d = np.array([self._pos[str(cf_id)][0], self._pos[str(cf_id)][1], self.takoff_alt])
                self.update_pos(str(cf_id), position_d)
            if count == self.number_of_agents:
                self.takeoffed = True
        
        # Check all cfs reached their initial points
        elif not self.reached_1st:
            count = 0
            for cf_id in self.cf_ids:
                if np.linalg.norm(self._pos[str(cf_id)] - self.initials[str(cf_id)]) < 0.1:
                    count += 1
                position_d = np.array([self.initials[str(cf_id)][0], self.initials[str(cf_id)][1], self.initials[str(cf_id)][2]])
                self.update_pos(str(cf_id), position_d)
            if count == self.number_of_agents:
                self.reached_1st = True
                print('Initial points reached!!')
                raw_input("Press Enter to continue...") # use input() for python3

        # flocking path planning
        else:
            # call flocking and get the desired position of next timestep
            for cf_id in self.cf_ids:
                other_cfs = self.cf_ids[:]
                other_cfs.remove(cf_id)
                self._dist_to_goal[str(cf_id)] = np.linalg.norm(self._pos[str(cf_id)] - self.finals[str(cf_id)])
                force = -self.c1*self._vel[str(cf_id)] - self.c2*(self._pos[str(cf_id)] - self.finals[str(cf_id)])
                for other_cf in other_cfs:
                    dist_v = self._pos[str(other_cf)] - self._pos[str(cf_id)]
                    dist = np.linalg.norm(dist_v)
                    d = 2*self.radius + self.d_star
                    CommunicationRadious = np.cbrt(3*np.square(self.MaxVelo)/(2*self.RepulsiveGradient)) + d
                    if dist < CommunicationRadious:
                        ForceComponent = -self.RepulsiveGradient * np.square(dist - CommunicationRadious)
                        force += ForceComponent * (dist_v)/dist                
                velocity_d = self._vel[str(cf_id)] + force * self.dt
                position_d = self._pos[str(cf_id)] + velocity_d*self.dt
                position_d[2] = 1.0 # for 2D sim
                self.update_pos(str(cf_id), position_d)

            # Check all cfs reached their final points
            if self._dist_to_goal[max(self._dist_to_goal)] < 0.2:
                self.flag['preland'] = 1

    def iteration(self, event):

        # publish goal messages for each uav
        self.publish_msg()

        # publish rotor velocities for each uav
        for id in enumerate(self.cf_ids):
            self.update_rotor_vels(id)


if __name__ == '__main__':
    # write code to create PositionControllerNode_ChihChun

    rospy.init_node('position_controller_node_ChihChun_flocking', disable_signals=True)
    ids = [1,2]
    initials = np.array([[0,0,0],[1,1,0]])
    finals = np.array([[1,1,1],[0,0,1]])
    vx_ds = [0.0,0.0]# desired vx
    vy_ds = [0.0,0.0]# desired vy
    vz_ds = [0.0,0.0]# desired vz
    yaw_ds = [0.0,0.0]# desired yaw
    dt = 1.0/15
    cf_flocking = position_controller_flock_node(ids, initials, finals, vx_ds, vy_ds, vz_ds, yaw_ds)
    rospy.Timer(rospy.Duration(dt), cf_flocking.iteration)
    rospy.spin()

