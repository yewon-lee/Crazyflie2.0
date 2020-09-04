#!/usr/bin/env python2

"""
ROS Node for controlling the CrazyFlie 2.0.

This ROS node subscribes to the following topics:
/crazyflie2/command/trajectory
/crazyflie2/odometry

This ROS node publishes to the following topics:
/crazyflie2/command/motor_speed

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

import csv
import os

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from position_controller import PositionController
from std_msgs.msg import Empty, String

# Import classes - addendum

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # subscribe
from nav_msgs.msg import Odometry # subscribe
from mav_msgs.msg import Actuators # publish
from roll_pitch_yawrate_thrust_crazyflie_swarm import roll_pitch_yawrate_thrust_crazyflie_swarm # change?

# NOTES:
# Check whether subscribes to desired position
# check if imported classes can be imported or are needed


class PositionControllerNode_ChihChun(object):
    """ROS interface for controlling the Crazyflie 2 in Gazebo."""
    # write code here to define node publishers and subscribers
    # publish to /crazyflie2/command/motor_speed topic
    # subscribe to /crazyflie2/command/trajectory and /crazyflie2/odometry

    def __init__(self):
        
        # Publishers: rotor velocities
        self.pub_rotor_vel = rospy.Publisher('/crazyflie2/command/motor_speed', Actuators, queue_size=1)
        
        # Subscribers: desired posn, current posn

        #self._goal_msg = MultiDOFJointTrajectory()
        #self.sub_goal = rospy.Subscriber('crazyflie2/command/trajectory', MultiDOFJointTrajectory, self._goal_callback)
        
        self._currpos_msg = Odometry()
        self.sub_currpos = rospy.Subscriber('/crazyflie2/odometry', Odometry, self._currpos_callback)

        # end of publisher/subscriber
        # initialize other parameters

        self.data_list = [['Time', 'Desired_x', 'Desired_y', 'Desired_z',
                           'Actual_x', 'Actual_y', 'Actual_z',
                           'Desired_yaw', 'Desired_roll', 'Desired_pitch', 'Desired_yaw_r', 'Desired_climb_r',
                           'Actual_roll', 'Actual_pitch', 'Actual_yaw_r', 'Actual_climb_r']]     

        self.start = [1.0, 1.0]

        self.wp_id = 1
        self.previous_time = 0.0
        self.change_time = -100.0

        self.controller = PositionController()
        self.init_time = rospy.get_time()

        # initialize position and roll, pitch, yaw
        self._x = 0
        self._y = 0
        self._z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self._z_old = 0.0
        self._z_oold = 0.0

        self.yaw_old = 0.0


    def set_rotor_vel(self, pitch_c, roll_c, yaw_rate_c, p, q, r, roll, pitch, yaw, thrust): # change arg names?

        # get conversions
        rotorvel_converter = roll_pitch_yawrate_thrust_crazyflie_swarm(pitch_c, roll_c, yaw_rate_c, p, q, r, roll, pitch, yaw, thrust)
        rotor_velocities = rotorvel_converter.CalculateRotorVelocities()

        # publish rotor velocities to Actuator
        msg = Actuators()
        msg.angular_velocities = rotor_velocities
        msg.header.stamp = self._currpos_msg.header.stamp
        self.pub_rotor_vel.publish(msg)

    def _currpos_callback(self, msg):
        self._currpos_msg = msg
    
    def get_data(self):
        self._z_oold = self._z_old
        self._z_old = self._z

        self.yaw_old = self.yaw

        # self._pos = self._vicon_msg.transform.translation
        self._x = self._currpos_msg.pose.pose.position.x
        self._y = self._currpos_msg.pose.pose.position.y
        self._z = self._currpos_msg.pose.pose.position.z

        # Convert to quaternion object for use by euler_from_quaternion()
        quaternion = np.array([self._currpos_msg.pose.pose.orientation.x,
                               self._currpos_msg.pose.pose.orientation.y,
                               self._currpos_msg.pose.pose.orientation.z,
                               self._currpos_msg.pose.pose.orientation.w])

        # Determine the euler angles 
        euler = euler_from_quaternion(quaternion) # change? can i use this?
        self.roll = euler[0]
        self.pitch = euler[1]
        self.yaw = euler[2]

        # Determine the euler angular rates p,q,r and thrust
        self.p = self._currpos_msg.twist.twist.angular.x
        self.q = self._currpos_msg.twist.twist.angular.y
        self.r = self._currpos_msg.twist.twist.angular.z
        #self.thrust = self._currpos_msg.twist.twist.linear.z

        self.x_d = 1.0
        self.y_d = 1.0
        self.z_d = 1.0
        self.yaw_d = 0.0

        self.vx_d = 0.0
        self.vy_d = 0.0
        self.vz_d = 0.0

        # desired position
        #self.x_d = self._goal_msg.points.transforms.translation.x
        #self.y_d = self._goal_msg.points.transforms.translation.y
        #self.z_d = self._goal_msg.points.transforms.translation.z

        # desired orientation
        #self.quat = np.array([self._goal_msg.points.transforms.rotation.x,
        #                      self._goal_msg.points.transforms.rotation.y,
        #                      self._goal_msg.points.transforms.rotation.z,
        #                      self._goal_msg.points.transforms.rotation.w])
        #euler = euler_from_quaternion(self.quat)
        #self.yaw_d = euler[2]

        # desired velocities
        #self.vx_d = self._goal_msg.points.velocities.linear.x # may need to change in future
        #self.vy_d = self._goal_msg.points.velocities.linear.y
        #self.vz_d = self._goal_msg.points.velocities.linear.z

    def iteration(self, event):
        current_time = rospy.get_time() - self.init_time
        dt = current_time - self.previous_time
        self.previous_time = current_time
        #print(dt)

        if dt == 0:
            dt = 0.01
    
        act_climb_r = (self._z - 2 * self._z_old + self._z_oold) / (dt**2)
        print("z_dd_real: ",act_climb_r)
        #act_yaw_r = (self.yaw - self.yaw_old)/dt

        self.get_data()

        roll_c, pitch_c, z_dot_c, yaw_dot_c = self.controller.get_command(
            self._x, self._y, self._z, self.roll, self.pitch, self.yaw,
            self.x_d, self.y_d, self.z_d, self.vx_d, self.vy_d, self.vz_d, self.yaw_d)

        #self.set_vel(roll_c, pitch_c, z_dot_c, yaw_dot_c)
        self.thrust_c = z_dot_c #+ 12000
        print("thrust: ", z_dot_c)
        self.set_rotor_vel(pitch_c, roll_c, yaw_dot_c, self.p, self.q, self.r, self.roll, self.pitch, self.yaw, self.thrust_c)

        # print(self.wp_id)
        # print(self.landm_num)
        # # print('time interval: ', dt)
        # # print('current time: ', current_time)
        # print("actual pos are: ", self._x, self._y, self._z)
        # print("desired pos are: ", self.x_d, self.y_d, self.z_d)
        # print("desired and actual yaw: ", self.yaw_d, self.yaw)
        # # # print("velocity are: ", self.vx_d, self.vy_d, self.vz_d)
        # # # print('roll is: ', roll_c, 'pitch is: ', pitch_c, 'climb rate: ', z_dot_c, 'yaw dot: ', yaw_dot_c)
        # print('\n')

if __name__ == '__main__':
    # write code to create PositionControllerNode_ChihChun
    rospy.init_node('position_controller_node_ChihChun_swarm', disable_signals=True)

    ''' Setup position and orientation for planning '''

    crazyflie2 = PositionControllerNode_ChihChun()
    try:
        rospy.Timer(rospy.Duration(1.0/100), crazyflie2.iteration)
        rospy.spin()
    except:
        crazyflie2.set_rotor_vel(0,0,0,0,0,0,0,0,0,0)
        rospy.spin()

