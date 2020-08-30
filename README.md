# Crazyflie2.0
This is an extension of CrazyS, originally developed by @gsilano (see https://github.com/gsilano/CrazyS for details). A few innovations include:
- Using a new position controller that allows the Crazyflie to move in the x-y direction, and not just the y direction
- Integrating the FMP algorithm with the controller
- The above can be done with single or multiple Crazyflies

Files in my github not found in @gislano's:
- rotors_control/src/nodes/position_controller_node_ChihChun.py
- rotors_control/src/nodes/position_controller_node_ChihChun_1.py
- rotors_control/src/nodes/position_controller_node_ChihChun_2.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking_1.py
- rotors_control/src/nodes/position_controller_node_ChihChun_flocking_2.py

In the following sections, I demonstrate the setup and elaborate on the different scenarios that can be simulated in Gazebo.

## Set-up
I recommend setting up my code following @gsilano's instructions on https://github.com/gsilano/CrazyS. However, make sure to substitute my github instead of his if you want to use my version of the code (mine is a slight extension of his, as it includes a couple more files).


## Simulating single-UAV trajectory tracking
This simulation takes a single Crazyflie from position (0,0,0) to (1,1,1) using a position controller. To launch the single-UAV trajectory tracking example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_hovering_example_ChihChun.launch
```
It is possible to change the final position of the Crazyflie. 



## Simulating the double-UAV trajectory tracking
This simulation uses the same position controller as the single-UAV trajectory tracking case. By default, it takes two Crazyflies from (-2,0,0) and (-1,0,0) to (-1,1,1) and (0,1,1), respectively. To launch the double-UAV trajectory tracking example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_swarm_example_ChihChun.launch
```

The 

## Simulating double-UAV system with the FMP algorithm
This simulation uses the same position controller as the single- and double-UAV trajectory tracking cases. By default, it takes two Crazyflies from () and () to () and () without the two UAVs colliding with one another, following the FMP algorithm. To launch this example, type the following into your terminal:

```bash
roslaunch rotors_gazebo crazyflie2_multiUAV_flocking_ChihChun.launch
```

