Cyton Gamma 300 (GazeboSim/RealRobot + MoveIt) 
============================================

1. [Description](#description)
2. [Packages](#packages)
3. [Dependencies](#dependencies)
4. [Quick start](#quickstart)
5. [Tutorial](#tutorial)
6. [Related Sources](#related)
7. [Rationale](#rationale)
8. [Maintainer](#maintainer)


### <a name="description"></a>1. Description

Yet another repository that holds a collection of [ROS](http://www.ros.org/) 
packages to simulate and actuate
the [Cyton Gamma 300](http://robots.mobilerobots.com/wiki/Cyton_Gamma_300_Arm)
7-DOF robotic arm. 
The simulation is done in [Gazebo](http://gazebosim.org/) and 
the [MoveIt!](http://moveit.ros.org/) is used as a motion planning framework.
The code has been tested with the [kinetic](http://wiki.ros.org/kinetic) 
distribution of ROS (as of mid March 2017).


### <a name="packages"></a>2. Packages

* `cyton_gamma_300_description`: [xacro](http://wiki.ros.org/xacro) description 
   of the [URDF](http://wiki.ros.org/urdf) robot model;

* `cyton_gamma_300_gazebo`: gazebo simulation of the robot;

* `cyton_gamma_300_controllers`: controllers for actuating gazebo model or
   the real robot using MoveIt framework;

* `cyton_gamma_300_moveit`: configuration files of the 
   setup assistant to enable MoveIt functionality.

The aforementioned `cyton_gamma_300_*` packages enable 
motion planning for the Cyton Gamma 300 arm. 
These packages are **not** self-contained and the dependencies should
be met to successfully run the software.

### <a name="dependencies"></a>3. Dependencies

First, one needs a full installation of ROS, otherwise 
some other packages might be missing 
(refer [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) for the full ROS
installation instructions). 
Second, one needs to install [MoveIt](http://moveit.ros.org/install/). 
Third, for sensing one needs to install OpenCV 
(refer [here](http://milq.github.io/install-opencv-ubuntu-debian/) for
the installation instructions).

* [`dynamixel_motor`](http://wiki.ros.org/dynamixel_motor)

* [`ros_control`](http://wiki.ros.org/ros_control)

* [`ros_controllers`](http://wiki.ros.org/ros_controllers)

* [`control_toolbox`](http://wiki.ros.org/control_toolbox)

* [`realtime_tools`](http://wiki.ros.org/realtime_tools)

* [`warehouse_ros`](http://wiki.ros.org/warehouse_ros)

### <a name="quickstart"  ></a>4. Quick start: Plan & Execute

1. In Gazebo simulation:

`roslaunch cyton_gamma_300_controllers gazebo_moveit.launch`

2. On the real robot:

`roslaunch cyton_gamma_300_controllers robot_moveit.launch`


### <a name="tutorial"    ></a>5. Tutorial

1. Visualize the URDF model in RViz:

`roslaunch cyton_gamma_300_description urdf_in_rviz.launch`

2. Spawn the URDF model in Gazebo:

`roslaunch cyton_gamma_300_gazebo gazebo_world.launch`

The model is subject to gravity forces and falls down from its original position.

3. Test different types of 
[ros\_controllers](https://github.com/ros-controls/ros_controllers)
in Gazebo simulation:

* E.g. `JointPositionController` from `effort_controllers`:

`roslaunch cyton_gamma_300_controllers gazebo_effort_controllers.launch`

* Or another example of `JointPositionController` from `position_controllers`:

`roslaunch cyton_gamma_300_controllers gazebo_position_controllers.launch`

* `JointTrajectoryController` from `position_controllers` allows to control groups 
of joints:

`roslaunch cyton_gamma_300_controllers gazebo_joint_trajectory_controllers.launch`

4. Specify a target pose in [RViz](http://wiki.ros.org/rviz), plan in 
[MoveIt!](http://moveit.ros.org/) using [OMPL](http://ompl.kavrakilab.org/) 
and execute a plan in [Gazebo](http://gazebosim.org/):

`roslaunch cyton_gamma_300_controllers gazebo_moveit.launch`

5. Run MoveIt on the actual robot:

* Make sure you are in the
[dialout](http://askubuntu.com/questions/58119/changing-permissions-on-serial-port) 
group or you have read/write access to `/dev/ttyUSB0` 
(assuming that `/dev/ttyUSB0` is your 
[dynamixel bus](http://support.robotis.com/en/product/auxdevice/interface/usb2dxl_manual.htm) 
address).
The following launch file will start everything one needs to plan and
execute the plan on the actual robot:

`roslaunch cyton_gamma_300_controllers robot_moveit.launch`

* Alternatively, it is possible to launch the same functionality
  separately (e.g. for debugging purposes):

First, we need to run a `controller_manager.py` script from `dynamixel` 
package that queries the stepper motors on the bus and initializes them. 
We logically separate motors 0-6 for the arm (or the *manipulator* planning group) 
and the stepper motor 7 for the gripper (the *gripper* planning group). 
We start initialization with the manipulator planning group:

`roslaunch cyton_gamma_300_controllers robot_manipulator_manager.launch` 

Second, we need to spawn controllers that would activate the
manipulator joints (i.e. motors 0-6 on the bus):

`roslaunch cyton_gamma_300_controllers robot_manipulator_controller_spawner.launch`

Third, we need to repeat steps one and two for the gripper:

`roslaunch cyton_gamma_300_controllers robot_gripper_manager.launch` 

`roslaunch cyton_gamma_300_controllers robot_gripper_controller_spawner.launch`

Finally, we bring up MoveIt:

`roslaunch cyton_gamma_300_controllers robot_moveit_movegroup.launch`




### <a name="related"     ></a>6. Related Sources

### <a name="rationale"   ></a>7. Rationale

As already quite a lot of related sources exist on the
[github](https://github.com/search?utf8=%E2%9C%93&q=cyton+gamma+300), 
the goal is 
(i) provide a functional step-by-step tutorial, 
(ii) to document the working setup for the current version of ROS.

### <a name="maintainer"  ></a>8. Maintainer

[Konstantin Selyunin](http://selyunin.com/), for
suggestions/questions/comments please contact: selyunin [dot] k [dot] v [at] gmail [dot] com
