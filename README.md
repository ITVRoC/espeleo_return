# espeleo_return
----------------------
This repository contains the code for the local autonomous navigation for radio connection reestablishment.


## Script included on this package:
- autonomous_return.py: This script is responsible to record the path traveled by the robot in an array/buffer and also for sending this array to the script vec_field_control.py located in espeleo_control package, when the radio connection is lost.


## How to interact

When the radio connection is lost, this code implements the algorithm that makes the robot return autonomously to a certain point known to have a radio connection. In order to run the code properly, it is needed to subscribe to the position of the robot. The code is implemented using position data from the topic /tf. When used in a simulation, a integrated interface can be utilized by the user to control the status of the communication.

With roscore and coppeliasim running (in this order), run the following launch files:

`roslaunch espeleo_vrep_simulation espeleo_sim.launch`
`roslaunch espeleo_teleop keyboard.launch`
`roslaunch espeleo_return return.launch`

With everything running, you can control the robot to a certain point and then use the interface to turn off the communication, thus making the robot return the path followed. You can also turn the communication back on, to keep controlling the robot to another point.


**Topics:**
Published:
- `/espeleo/traj_points` (message type:`geometry_msgs.msg/Polygon`)
- `/visualization_marker_array` (message type:`visualization_msgs.msg/MarkerArray`)
- `/flag/signal` (message type:`std_msgs.msg/Bool`)
- `/espeleo/vecfield_enable` (message type:`std_msgs.msg/Bool`)

Subscribed:
- `/flag/signal` (message type:`std_msgs.msg/Bool`)
- `/tf` (message type:`tf2_msgs/TFMessage`)

**Launch file**
- `return.launch`


## Input parameters

The following parameters are found in the file parameters.yaml, inside the directory espeleo_return/config:
- d_displacement_limit: Minimum variance in the position of the robot to record the new position.
- d_tolerance: Tolerance between the final point of stop set by the array and the actual position of the robot.
- d_return: Maximum distance to return, stored in the array in meters.
- d_intermediate: Intermediate distance to return repeatedly.
- comms_interface_enable: Interface used to control the communication state in simulations.
- odometry: Topic used to obtain information about the position of the robot.
- child_frame_id: Transform which contains the robot pose.

There is also another .yaml file, control_params.yaml, to adjust parameters of the vec_field_control.py script used to control the robot once it has lost communication. For more details about these parameters, check the espeleo_control package.
