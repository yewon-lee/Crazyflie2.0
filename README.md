# Crazyflie2.0
This is an extension of CrazyS, originally developed by @gsilano (see https://github.com/gsilano/CrazyS for details). A few innovations include:
- Using a new position controller that allows the Crazyflie to move in the x-y direction, and not just the y direction
- Integrating the FMP algorithm with the controller
- The above can be done with single or multiple Crazyflies

Files in my github not found in @gsilano's:
- rotors_control/src/nodes/position_controller_node_ChihChun.py
- rotors_control/src/nodes/position_controller_node_ChihChun_1.py
- rotors_control/src/nodes/position_controller_node_ChihChun_2.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking_1.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking_2.py
- rotors_conrol/src/nodes/position_controller_node_ChihChun_swarm.py
- rotors_gazebo/launch/crazyflie2_hovering_example_ChihChun.launch
- rotors_gazebo/launch/crazyflie2_multiUAV_flocking_ChihChun.launch
- rotors_gazebo/launch/crazyflie2_singleUAV_flocking_ChihChun.launch
- rotors_gazebo/launch/crazyflie2_swarm_example_ChihChun.launch

In the following sections, I demonstrate the setup and elaborate on the different scenarios that can be simulated in Gazebo.

## Set-up
I recommend setting up my code following @gsilano's instructions on https://github.com/gsilano/CrazyS. However, make sure to substitute my github instead of his if you want to use my version of the code (mine is a slight extension of his, as it includes a couple more files).


## Simulating single-UAV trajectory tracking

### Launching

This simulation takes a single Crazyflie from position (0,0,0) to (1,1,1) using a position controller. To launch the single-UAV trajectory tracking example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_hovering_example_ChihChun.launch
```

### Observations

A plot of what should be observed for this simulation is shown below. The following is a position versus time plot of the Crazyflie when the simulation is launched. The y-axis is the position (in x, y and z directions) in meters and the x-axis is the time in seconds. The x and y positions converge relatively well, but there is a lot of oscillation in the z direction.

![a-singleUAV](https://user-images.githubusercontent.com/68444609/91673961-85ba1380-eaeb-11ea-8222-36530fc9acac.png)

### Customizing

It is possible to change the final desired position, velocity, and yaw of the Crazyflie. To customize these, change the following variables in rotors_control/src/nodes/position_controller_node_ChihChun.py:

```python
  self.x_d = 1.0 # desired x
  self.y_d = 1.0 # desired y
  self.z_d = 1.0 # desired z
  self.yaw_d = 0.0 # desired yaw

  self.vx_d = 0.0 # desired x-velocity
  self.vy_d = 0.0 # desired y-velocity
  self.vz_d = 0.0 # desired z-velocity
```

### Controller Tuning
There are two sets of parameters and conroller gains that can be tuned.

The first is those of the position controller (also known as the "reference generator") itself, which takes in odometry information to calculate the roll, pitch, yaw rate, and thrust commands.file contains the position controller. The reference generator is found in rotors_control/src/nodes/position_controller.py, and the following shows the controller gains and parameters that can be tuned (these can be found in the _init_ function):

``` python

self.Kp_yaw = 4

# natural frequencies and damping ratios
w_n_x = 3.0  
zeta_x = 0.0 
w_n_y = 3.0
zeta_y = 1.0
w_n_z = 10.0 
zeta_z = 1.0 

# gains
self.Kp_xc = w_n_x**2
self.Kv_xc = 2 * w_n_x * zeta_x
self.Kp_yc = w_n_x**2
self.Kv_yc = 2 * w_n_x * zeta_x
self.Kp_zc = w_n_z**2
self.Kv_zc = 2 * w_n_z * zeta_z

# x, y and z limiters
self.limit_x = 1.0 
self.limit_y = 1.0 
self.limit_z = 1.0 
```

The second is the on-board controller which converts the roll, pitch, yaw rate, and thrust commands to the four rotor velocities that get sent to Gazebo. This can be found in the file, rotors_control/src/nodes/roll_pitch_yawrate_thrust_crazyflie.py. The following lists the parameters and control gains that may be tuned:

``` python

#  parameters
self.ANGULAR_MOTOR_COEFFICIENT = 1.8 # ANGULAR_MOTOR_COEFFICIENT
self.MOTORS_INTERCEPT = 2300 # MOTORS_INTERCEPT [rad/s]
self.MAX_PROPELLERS_ANGULAR_VELOCITY = 2618 # MAX PROPELLERS ANGULAR VELOCITY [rad/s]

# controller gains
self.attitude_gain_kp = [0.0611, 0.0611]
self.attitude_gain_ki = [0.0349, 0.0349]
self.rate_gain_kp = [1000,1000,1000]
self.rate_gain_ki = [0.0, 0.0, 95.6939] 

# thrust percentage
self.thrust_percentage = 1.0
```

The above can all be found in the _init_ function of the roll_pitch_yawrate_thrust_crazyflie.py file.

### Limitations

There are currently a few limitations of this simulation:
- Although the x- and y- positions converge relatively well, there is a lot of z-direction oscillation which does not dampen in time (but it doesn't grow, either).
- I have not simulated this test case with other final positions, yaws, and velocities - so this is potentially something to investigate further.


## Simulating the double-UAV trajectory tracking

### Launching

This simulation uses the same position controller as the single-UAV trajectory tracking case. By default, it takes two Crazyflies from (-2,0,0) and (-1,0,0) to (-1,1,1) and (0,1,1), respectively. To launch the double-UAV trajectory tracking example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_swarm_example_ChihChun.launch
```

### Observations

Plots of what should be observed for this simulation are shown below. Although it uses the exact same position controller and controller gains as the single-UAV trajectory tracking example, there are a few noticable differences in the response plots:
- The z-direction oscillations are minimal.
- The positions converge faster.

It can also be noted that there is some error in the z-direction.

The following is a position versus time plot of crazyflie2_1 (i.e. the first of the two UAVs).

![a-UAV1plot](https://user-images.githubusercontent.com/68444609/91673964-88b50400-eaeb-11ea-8f13-74cce74fdf5e.png)

The following is a position versus time plot of crazyflie2_2 (i.e. the second of the two UAVs).

![a-UAV2plot](https://user-images.githubusercontent.com/68444609/91673967-8baff480-eaeb-11ea-8e06-eac0014b0b96.png)



### Customizing

It is possible to customize the two Crazyflies' initial and final positions. The Crazyflies' intial positions can be changed directly in the launch file, found in rotors_gazebo/launch/crazyflie2_swarm_example_ChihChun.launch:

```launch
<!-- Set the initial position -->
<arg name="x" value="-2.0"/>
<arg name="y" value="0.0"/>
```

The Crazyflies' final positions, yaws, and velocities can be changed by referring to their respective position controller node files: rotors_control/src/nodes/position_controller_node_ChihChun_1.py and rotors_control/src/nodes/position_controller_node_ChihChun_2.py. Change the following variables: 

```python
  self.x_d = 1.0 # desired x
  self.y_d = 1.0 # desired y
  self.z_d = 1.0 # desired z
  self.yaw_d = 0.0 # desired yaw

  self.vx_d = 0.0 # desired x-velocity
  self.vy_d = 0.0 # desired y-velocity
  self.vz_d = 0.0 # desired z-velocity
```

This trajectory-tracking double-UAV simulation can be extended to more UAVs, although because of the current structure of the code, it requires tedious modifications. Specifically, a separate position controller node must be created for each inividual Crazyflie (for instance, if you want a third Crazyflie, you would need to create a position_controller_node_ChihChun_3.py and include that in the launch file).

### Controller Tuning

This simulation uses the same controllers as those of the single-UAV simulation. Please refer to the "Controller Tuning" subsection in the single-UAV case.

### Limitations

Current limitations of this double-UAV trajectory tracking example are:
- The simulation is very slow with two UAVs; extending it to more UAVs will likely be even slower.
- Extending this simulation to more UAVs is a little tedious due to the structure of the code.
- The simulation has not been run with differing initial and final positions from the default ones. This is something that should be investigated further.

## Simulating double-UAV system with the FMP algorithm

### Launching

This simulation uses the same position controller as the single- and double-UAV trajectory tracking cases. By default, it takes two Crazyflies from (0,0,0) and (1,1,0) to (1,1,1) and (0,0,1) without the two UAVs colliding with one another, following the FMP algorithm. To launch this example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_multiUAV_flocking_ChihChun.launch
```

### Customizing
The two Crazyflies' initial and final desired positions can be easily customized. It simply involves changing the following lines in /rotors_control/src/nodes/position_controller_node_ChihChun_flocking.py, which can be found in the "main function":

``` python
initials = np.array([[0,0,0],[1,1,0]])
finals = np.array([[1,1,1],[0,0,1]])
vx_ds = [0.0,0.0] # value in index 0 and 1 is the desired vx for crazyflie_0 and crazyflie_1, respectively
vy_ds = [0.0,0.0] # value in index 0 and 1 is the desired vy for crazyflie_0 and crazyflie_1, respectively
vz_ds = [0.0,0.0] # value in index 0 and 1 is the desired vz for crazyflie_0 and crazyflie_1, respectively
yaw_ds = [0.0,0.0] # value in index 0 and 1 is the desired vx for crazyflie_0 and crazyflie_1, respectively
```

This simulation can also be extended to more UAVs. The current /rotors_control/src/nodes/position_controller_node_ChihChun_flocking.py supports up to four Crazyflies, but more can be added.

### Limitations
- There are still several bugs that need to be fixed before this simulation is fully functional. 

