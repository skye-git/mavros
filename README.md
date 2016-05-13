MAVROS
======

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

MAVLink extendable communication node for ROS.


mavros package
--------------

It is the main package, please see it's README.


mavros\_extras package
----------------------

This package contain some extra nodes and plugins for mavros, please see it's README.


libmavconn package
------------------

This package contain mavconn library, see it's README.
LibMAVConn may be used outside of ROS environment.


test\_mavros package
--------------------

This package contain hand-tests and [manual page][test] for APM and PX4 SITL.
Please see [README][test] first!


mavros\_msgs package
--------------------

This package contain messages and services used in mavros.


Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture requests on:

- [px4users Google Group (Mailing List) ](https://groups.google.com/forum/#!forum/px4users)
- [Mavros on Gitter IM](https://gitter.im/mavlink/mavros)
- [PX4/Firmware on Gitter IM](https://gitter.im/PX4/Firmware)

We'd like to keep the project bugtracker as free as possible, so please contact via the above methods. You can also PM us via Gitter.


## Skye Hardware In The Loop
It is possible to test Skye's firmware with a hardware in the loop simulation using mavros as a link between mavlink and ros-gazebo. The current tutorial has been tested with ROS Indigo.

### Requirements
Please follow the ReadMe tutorial of [skye_gazebo_simulation](https://github.com/skye-git/skye_gazebo_simulation) before proceding with the installation of mavros.

### Mavlink-ROS Package And Skye Dialect
You will be using the ROS python tools wstool, rosinstall,and catkin_tools for this installation. While they may have been installed during your installation of ROS you can also install them with:
```bash 
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```
First get the most recent copy of Skye's dialect:
```bash
mkdir -p ~/skye-git
cd ~/skye-git
git clone https://github.com/skye-git/mavlink
```
Now you're ready to get a copy of the source file of mavlink-ros package.

**Warning**: you should have already created a catkin workspace named "catkin\_ws" when you followed [skye Gazebo Simulation](https://github.com/skye-git/skye_gazebo_simulation/tree/indigo-devel)
```bash
cd ~/catkin_ws
wstool init ~/catkin_ws/src
rosinstall_generator mavlink | tee -a /tmp/mavlink.rosinstall
wstool merge -t src /tmp/mavlink.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src -y
```
Copy Skye's dialect into mavlink message_definition folder and compile it:
```bash
cp ~/skye-git/mavlink/message_definitions/v1.0/skye.xml ~/catkin_ws/src/mavlink/message_definitions/v1.0/skye.xml
cd ~/catkin_ws
catkin build
```
It is conviniente to source the setup.* file
```bash
source ~/catkin_ws/devel/setup.sh
```
Now you can download the mavros package and sapcenav drivers and compile them. 
**Warning:** you must perfom the following actions in the same workspace used for the installation of "skye_gazebo_simulations", that is assumed to be "~/catkin_ws".
```bash
cd ~/catkin_ws/src/
git clone https://github.com/skye-git/mavros -b skye_hil
git clone https://github.com/skye-git/joystick_drivers
cd ~/catkin_ws
 #compile using skye dialect from mavlink package
catkin build --cmake-args -DMAVLINK_DIALECT=skye
```

#### Troubleshooting
In case of `CMake Error: Could not find a package configuration file provided by "control_toolbox"`, please install the following packages:
```bash
sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
```

###Usage
To launch the HIL simulation identify to which USB port Mavlink is communicating and then type in a new terminal
```bash
roslaunch mavros skye_hil.launch fcu_url:=/dev/ttyUSB0:115200
```
In the above example Mavlink is using USB0. This launch files starts Gazebo in pause. You can press the "play" button whenever you are ready to start the simulation.

####Changing Fimrware Parameter
During HIL simulation it is not possible to change onboard paramters via QGroundControl. The above list of services show how to change onboard parameters.

  * /skye_mr/set_att_c_mod sets the attitude control mode. Possible values: 0 (Manual), 1 (5DOF), 2 (6DOF);
  * /skye_mr/set_pos_c_mod sets the position control mode. Possible values: 0 (Manual), 1 (cascade pid);
  * /skye_mr/set_param to set any onboard parameters, either if it's integer or float.
  
Examples:

```bash
rosservice call /skye_mr/set_att_c_mod 0
rosservice call /skye_mr/set_pos_c_mod 1
rosservice call /skye_mr/set_param 'INTEGER_PARAM_NAME' '[PARAM_VALUE, 0.0]'
rosservice call /skye_mr/set_param 'FLOAT_PARAM_NAME' '[0, PARAM_VALUE]'
```


