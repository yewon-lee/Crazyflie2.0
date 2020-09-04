#!/usr/bin/env python2
"""Class for writing position controller."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
# from ros_interface import ROSControllerNode

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist

class PositionController(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab."""

    # write code here for position controller

    def __init__(self):

        self.g = -9.8
        self.Kp_yaw = 4

        # natural frequencies and damping ratios
        w_n_x = 1.8#3.0 # 2.2 / 1.8 # 1.8
        zeta_x = 1.0 #1.0 # 0.9
        w_n_y = 3.0
        zeta_y = 2.0
        w_n_z = 1.0 # 4.0 / 1.8 # 2
        zeta_z = 1.0 #1.0 # 0.7

        # gains
        self.Kp_xc = w_n_x**2
        self.Kv_xc = 2 * w_n_x * zeta_x
        self.Kp_yc = w_n_x**2
        self.Kv_yc = 2 * w_n_x * zeta_x
        self.Kp_zc = w_n_z**2
        self.Kv_zc = 2 * w_n_z * zeta_z

        self.limit_x = 1.0 #0.7
        self.limit_y = 1.0 #0.7
        self.limit_z = 1.0 #0.5

        self.old_time = rospy.get_time()

        self._x_old = 0.0
        self._y_old = 0.0

        self._z_old = 0.0
        self._z_oold = 0.0

        self.yaw_old = 0.0
        self.yaw_d = 0.0

    def get_command(self, _x, _y, _z, roll, pitch, yaw, _x_d, _y_d, _z_d,
                    _vx_d, _vy_d, _vz_d, _yaw_d):
	    
        self.yaw_d = _yaw_d
        current_time = rospy.get_time()
        dt = current_time - self.old_time
        if dt == 0:
            dt = 1.0/100
        self.old_time = current_time

        z_dd = (_z - 2 * self._z_old + self._z_oold) / (dt**2)

        # commanded specific force
        f = (z_dd - self.g) / (np.cos(pitch) * np.cos(roll))

        _vx = (_x - self._x_old) / dt
        _vy = (_y - self._y_old) / dt
        _vz = (_z - self._z_old) / dt

        if (_x_d - _x) > self.limit_x:
            _x_d = _x + self.limit_x
        elif (_x_d - _x) < -self.limit_x:
            _x_d = _x - self.limit_x

        if (_y_d - _y) > self.limit_y:
            _y_d = _y + self.limit_y
        elif (_y_d - _y) < -self.limit_y:
            _y_d = _y - self.limit_y

        if (_z_d - _z) > self.limit_z:
            _z_d = _z + self.limit_z
        elif (_z_d - _z) < -self.limit_z:
            _z_d = _z - self.limit_z

        # commanded horizontal accelerations
        x_dd_c = self.Kv_xc * (_vx_d - _vx) + self.Kp_xc * (_x_d - _x)
        y_dd_c = self.Kv_yc * (_vy_d - _vy) + self.Kp_yc * (_y_d - _y)
        z_dd_c = self.Kv_zc * (_vz_d - _vz) + self.Kp_zc * (_z_d - _z)
        print("x_dd_c: ",x_dd_c)
        print("y_dd_c: ",y_dd_c)
        print("z_dd_c: ",z_dd_c)

        # code for tuning
        #x_dd_c = 1
        #y_dd_c = 0
        #z_dd_c = 0

        if (-y_dd_c / f > 1):
            y_dd_c = -1.0*f
        elif (-y_dd_c/f < -1):
            y_dd_c = 1.0*f


        # commanded rolls and pitch
        roll_c_int = np.arcsin(-y_dd_c / f)

        if (x_dd_c / f / np.cos(roll_c_int) > 1):
            x_dd_c = 0.98*f *np.cos(roll_c_int) #0.98
        elif (x_dd_c/f / np.cos(roll_c_int) < -1):
            x_dd_c = -0.98*f*np.cos(roll_c_int) #-0.98


        pitch_c_int = np.arcsin(x_dd_c / f / np.cos(roll_c_int))
        roll_c = roll_c_int * np.cos(yaw) + pitch_c_int * np.sin(yaw)
        pitch_c = -roll_c_int * np.sin(yaw) + pitch_c_int * np.cos(yaw)

        # commanded climb and yaw rates
        climb_rate_c = z_dd_c

        # need to wrap angles for yaw so we dont get weird yaw rates
        # first wrap both angles from pi to - pi
        while self.yaw_d > np.pi:
            self.yaw_d = self.yaw_d - 2*np.pi
        while self.yaw_d < -np.pi:
            self.yaw_d = self.yaw_d + 2*np.pi
        while yaw > np.pi:
            yaw = yaw - 2*np.pi
        while yaw < -np.pi:
            yaw = yaw + 2*np.pi
        # then modify to get correct yaw rate
        if (self.yaw_d - yaw) > np.pi:
            yaw = yaw + 2*np.pi
        elif (self.yaw_d - yaw) < -np.pi:
            yaw = yaw - 2*np.pi
        yaw_rate_c = self.Kp_yaw * (self.yaw_d - yaw)
        #yaw_rate_c = 0.5
        #print("desired yaw: ", self.yaw_d)
        #print("actual yaw: ", yaw)

        # update old values
        self._x_old = _x
        self._y_old = _y
        self._z_oold = self._z_old
        self._z_old = _z
        # self.yaw_old = yaw

	    # output limiters
        # roll pitch and climb were [-1,1]
        # yaw rate was [-5,5]
#        if (roll_c > np.pi/6):
#            roll_c = np.pi/6
#        elif (roll_c < -np.pi/6):
#            roll_c = -np.pi/6

#        if (pitch_c > np.pi/6):
#            pitch_c = np.pi/6
#        elif (pitch_c < -np.pi/6):
#            pitch_c = -np.pi/6

#        if (climb_rate_c > 8163):
#            climb_rate_c = 8163
#        elif (climb_rate_c < 5156):
#            climb_rate_c = 5156

#        if (yaw_rate_c > 1.11*np.pi):
#            yaw_rate_c = 1.11*np.pi
#        elif (yaw_rate_c < -1.11*np.pi):
#            yaw_rate_c = -1.11*np.pi

        # print(roll_c, pitch_c, climb_rate_c, yaw_rate_c)

        return roll_c, pitch_c, climb_rate_c, yaw_rate_c
