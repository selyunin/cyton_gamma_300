Cyton Gamma 300 (GazeboSim/Real robot + Moveit) 
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

* `cyton_gamma_300_control`: controllers for actuating gazebo model or
   the real robot using MoveIt framework;

* `cyton_gamma_300_moveit`: configuration files of the 
   setup assistant to enable MoveIt functionality.

These are the core packages to enable motion planning for the Cyton
Gamma 300 arm. 
These packages are **not** self-contained and the dependencies should
be met to successfully run the software.

### <a name="dependencies"></a>3. Dependencies

### <a name="quickstart"  ></a>4. Quick start

### <a name="tutorial"    ></a>5. Tutorial

### <a name="related"     ></a>6. Related Sources

### <a name="rationale"   ></a>7. Rationale

### <a name="maintainer"  ></a>8. Maintainer
