#!/usr/bin/env python
'''
Created on Mar 20, 2017
@author: Konstantin Selyunin
'''
import roslaunch
import roslib
import rospy
import rospkg
import os
import time

class RobotControllerLauncher():

    def start_launchfile(self, filename):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [filename])
        launch.start()

    def __init__(self): 
        rospack = rospkg.RosPack()
        cyton_gamma_300_controllers_path = rospack.get_path('cyton_gamma_300_controllers')

        robot_manipulator_manager_launch_file = cyton_gamma_300_controllers_path + \
                                                "/launch/robot_manipulator_manager.launch"
        robot_gripper_manager_launch_file     = cyton_gamma_300_controllers_path + \
                                                "/launch/robot_gripper_manager.launch"
        robot_manipulator_spawner_launch_file = cyton_gamma_300_controllers_path + \
                                                "/launch/robot_manipulator_controller_spawner.launch"
        robot_gripper_spawner_launch_file     = cyton_gamma_300_controllers_path + \
                                                "/launch/robot_gripper_controller_spawner.launch"
        robot_moveit_launch_file              = cyton_gamma_300_controllers_path + \
                                                "/launch/robot_moveit_movegroup.launch"

        rospy.loginfo("Manipulator manager: starting...")
        self.start_launchfile(robot_manipulator_manager_launch_file)
        rospy.loginfo("Manipulator manager: done...")

        time.sleep(2)

        rospy.init_node('robot_controller_launcher', anonymous=True)

        rospy.loginfo("Manipulator controller spawner: starting...")
        self.start_launchfile(robot_manipulator_spawner_launch_file)
        rospy.loginfo("Manipulator controller spawner: done...")

        time.sleep(2)

        rospy.loginfo("Gripper manager: starting...")
        self.start_launchfile(robot_gripper_manager_launch_file)
        rospy.loginfo("Gripper manager: done...")

        time.sleep(2)

        rospy.loginfo("Gripper controller spawner: starting...")
        self.start_launchfile(robot_gripper_spawner_launch_file)
        rospy.loginfo("Gripper controller spawner: done...")

        time.sleep(2)

        rospy.loginfo("Robot MoveIt: starting...")
        self.start_launchfile(robot_moveit_launch_file)
        rospy.loginfo("Robot MoveIt: started...")


if __name__ == '__main__':
    try:
        r = RobotControllerLauncher()
        #while(True):
            #pass
        rospy.spin()
    except rospy.ROSInterruptException: pass
