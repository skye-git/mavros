MAVROS
======

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/mavlink/mavros?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

MAVLink extendable communication node for ROS.

- Since 2014-08-11 this repository contains several packages.
- Since 2014-11-02 hydro support splited from master to hydro-devel branch.
- Since 2015-03-04 all packages also dual licensed under terms of BSD license.
- Since 2015-08-10 all messages moved to mavros\_msgs package
- Since 2016-02-05 (v0.17) frame conversion changed again


mavros package
--------------

It is the main package, please see it's [README][mrrm].
Here you may read [installation instructions][inst].


mavros\_extras package
----------------------

This package contain some extra nodes and plugins for mavros, please see it's [README][exrm].


libmavconn package
------------------

This package contain mavconn library, see it's [README][libmc].
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
mkdir -p ~/skye-git/mavlink
cd ~/skye-git/mavlink
git clone https://github.com/skye-git/mavlink
```
Now you're ready to get a copy of the source file of mavlink-ros package:
```bash
mkdir -p ~/mavlink_ros_ws/src
cd ~/mavlink_ros_ws
catkin init
wstool init ~/mavlink_ros_ws/src
rosinstall_generator mavlink | tee -a /tmp/mavlink.rosinstall
wstool merge -t src /tmp/mavlink.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src -y
```
Copy Skye's dialect into mavlink message_definition folder and compile it:
```bash
cp ~/skye-git/mavlink/message_definitions/v1.0/skye.xml ~/mavlink_ros_ws/src/mavlink/message_definitions/v1.0/skye.xml
cd ~/mavlink_ros_ws
catkin build
```
It is conviniente to automatically source the setup.* file, generated by catkink, in every new shell:
```bash
echo "source ~mavlink_ros_ws/devel/setup.sh" >> ~/.bashrc
```
Now you can download the mavros package and compile it. 
**Warning:** you must perfom the following actions in the same workspace used for the installation of "skye_gazebo_simulations", that is assumed to be "~/catkin_ws".
```bash
cd ~/catkin_ws/src/
git clone https://github.com/skye-git/mavros -b skye_hil
cd ~/catkin_ws
 #compile using skye dialect from mavlink package
catkin_make --cmake-args -DMAVLINK_DIALECT=skye
```


