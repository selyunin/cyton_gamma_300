# Cyton Gamma 300 (GazeboSim/RealRobot + MoveIt)

1. [Description](#description)
1. [Packages](#packages)
1. [Installation](#installation)
    1. [Installation using docker](#installation-docker)
    1. [Installation on the host](#installation-host)
1. [Quick start](#quick-start)
    1. [Quick start using docker](#quick-start-docker)
    1. [Quick start on the host](#quick-start-host)
1. [Tutorial](#tutorial)
    1. [Tutorial using docker](#tutorial-docker)
    1. [Tutorial on the host](#tutorial-host)
1. [Related Sources](#related)
1. [Rationale](#rationale)
1. [Maintainer](#maintainer)


## <a name="description"/>Description

The project includes a collection of [ROS](https://www.ros.org/) 
packages to simulate and actuate the **Cyton Gamma 300** 7-DOF robotic arm from Robai 
(the company Robai is no longer active).
The simulation is done in [Gazebo](https://gazebosim.org/) and 
the [MoveIt](https://moveit.ros.org/) is used as a motion planning framework.
The code has been tested with the [melodic](https://wiki.ros.org/melodic) 
distribution of ROS (as of May 2023).

### <a name="packages"/>Packages

* [`cyton_gamma_300_controllers`](./src/cyton_gamma_300_controllers):
  controllers for actuating gazebo model or the real robot using MoveIt;

* [`cyton_gamma_300_description`](./src/cyton_gamma_300_description):
  [xacro](https://wiki.ros.org/xacro) description of the [URDF](https://wiki.ros.org/urdf) robot model;

* [`cyton_gamma_300_gazebo`](./src/cyton_gamma_300_gazebo):
  gazebo simulation of the robot;

* [`cyton_gamma_300_moveit`](./src/cyton_gamma_300_moveit):
  configuration files of the setup assistant to enable MoveIt functionality.

The aforementioned `cyton_gamma_300_*` packages enable motion 
planning for the Cyton Gamma 300 arm. 
These packages are **not** self-contained and the dependencies should
be met to successfully run the software.

### <a name="installation"></a>3. Installation

To run visualization / simulation / control, one needs to install
the dependencies, which the `cyton_gamma_300_*` packages require.
One can either use [docker](https://www.docker.com/) or install the dependencies on the host machine.
Docker is a recommended way of launching the code, 
as it encapsulates the dependencies and allows multiple ROS installations.

#### <a name="installation-docker"></a>3.1. Installation using Docker

The repo provides a [`Dockerfile`](./Dockerfile), from which the `docker image` with 
required pre-installed dependencies can be built.

Create the docker image with required dependencies:
```bash
make build
```
alternatively, on a system without make, run:
```bash
source .env && docker compose build cyton-gamma
```


#### <a name="installation-host"></a>3.2. Installation on the host

One needs to install:
* a full version of ROS, otherwise some other packages might be missing 
  (refer [here](https://wiki.ros.org/melodic/Installation/Ubuntu) for the full ROS
  installation instructions). 
* [MoveIt](http://moveit.ros.org/install/).
* OpenCV (refer [here](https://milq.github.io/install-opencv-ubuntu-debian/) for
the installation instructions).
 
In addition, the packages below must be found in your ROS workspace 
(or on your `$ROS_PACKAGE_PATH`):

* [`dynamixel_motor`](https://wiki.ros.org/dynamixel_motor) -- 
  [`cyton_gamma_300_controllers`](./src/cyton_gamma_300_controllers) 
  depends on the package to actuate the robot motors;

* [`ros_controllers`](https://wiki.ros.org/ros_controllers) -- are instantiated in 
  [`cyton_gamma_300_controllers`](./src/cyton_gamma_300_controllers);

* [`ros_control`](http://wiki.ros.org/ros_control) -- `ros_controllers` depend on this package;

* [`control_toolbox`](https://wiki.ros.org/ros_control) -- `ros_controllers` 
  depend on this package;

* [`realtime_tools`](https://wiki.ros.org/realtime_tools) -- `ros_controllers` dependency;

* [`warehouse_ros`](https://wiki.ros.org/warehouse_ros) -- required if
  one wants to use the warehouse database server while using motion
  planning (included by default when running `moveit setup_assistant`)

Fortunately, one can use available ROS tools to install missing
dependencies (replace `${WORKSPACE}` with a path to your catkin workspace and `${ROSDISTRO}` with the name
of the ROS distribution):

```bash
rosdep install --from-paths ${WORKSPACE} --ignore-src --rosdistro=${ROSDISTRO}`
```


### <a name="quick-start" />4. Quick start: Plan & Execute

#### <a name="quick-start-docker" />4.1. Quick start with docker

4.1.1. In Gazebo simulation:

```bash
make gazebo-moveit
```

4.1.2. On the real robot:

```bash
make robot-moveit
```

In the section below we describe the steps to launch the MoveIt on the host without docker.

#### <a name="quick-start-host" />4.2. Quick start on host


1. In Gazebo simulation:

```bash
roslaunch cyton_gamma_300_controllers gazebo_moveit.launch
```

2. On the real robot:

```bash
roslaunch cyton_gamma_300_controllers robot_moveit.launch
```

### <a name="tutorial"></a>5. Tutorial

#### <a name="tutorial-docker"></a>5.1. Tutorial using docker


1. Visualize the URDF model in RViz:

```bash
make urdf-in-rviz
```

2. Spawn the URDF model in Gazebo:

```bash
make urdf-in-gazebo
```

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



### <a name="tutorial"></a>5. Tutorial

1. Visualize the URDF model in RViz:

```bash
roslaunch cyton_gamma_300_description urdf_in_rviz.launch
```

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


### <a name="related"></a>6. Related Sources

[András Fekete](https://github.com/bandi13) pioneered in his 
[work](https://bandilabs.home.blog/2014/11/13/get-cyton-gamma-300-working-ros/) 
migrating Cyton Gamma 300 arm to open source software
from manufacturer's proprietary one. I must confess that I was not able to
run his code due to some errors, still he was a source of inspirations 
for the follow-up projects.
[Tyler Slabinski](https://github.com/Slabity) created beautiful
meshes of the Cyton Gamma 300 arm and wrote the URDF file that others
extensively use.
[AssistiveRoboticsUNH](https://github.com/AssistiveRoboticsUNH) and
[Andreas Lydakis](https://bitbucket.org/AndLydakis/) provided 
[here](https://github.com/AssistiveRoboticsUNH/cyton_gamma_300_ROS)
and 
[here](https://bitbucket.org/AndLydakis/cyton_gamma_300_ros) respectively
the repositories for running the actual robot using MoveIt, 
but did not include Gazebo support for simulations of Cyton Gamma 300.

### <a name="rationale"></a>7. Rationale

As already quite a lot of related sources exist on the
[github](https://github.com/search?utf8=%E2%9C%93&q=cyton+gamma+300), 
the goal is 
(i) provide a functional step-by-step tutorial, 
(ii) to document the working setup for the current version of ROS,
(iii) package dependencies in docker.

### <a name="maintainer"></a>8. Maintainer

[Dr. Konstantin Selyunin](https://selyunin.github.io/), for
suggestions/questions/comments please contact: selyunin [dot] k [dot] v [at] gmail [dot] com
