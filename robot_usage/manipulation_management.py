#! /usr/bin/env python

# Code taken from Robot Ignite Academy. If perfroms multiple movements in the joint task space. The arm
# will move from the home position to a position above a 40x40mm cube, the pick the cube up, return home,
# then put the cube bacl where it got it from.
import math
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import time
import numpy as np




def grab_object_at(roll,dist=0.13,z=0.15,pitch=0.6):
    """
    Functino that grabs a object at positoin x,y,z and returns arm bach to a default position
    :param x: x position of object relative to robot_coords
    :param y: y position of object relative to robot_coords
    :param z: height at wich OTM will be grabbed
    :return:
    """
    roll=roll+90
    x=math.sin(math.radians(roll))*dist+0.08
    y=math.cos(math.radians(roll))*dist
    target_pose=get_quaternion_pose(x,y,z,pitch)
    open_gripper()
    take_pose(target_pose)
    close_gripper()
    move_home()

def put_down_object_at(roll,dist=0.13,z=0.15,pitch=0.6):
    """
    Functino that puts down object at positoin x,y,z and returns arm bach to a default position
    :param roll: angle where the object should be put down (from +90 to -90)
    :param dist: dsitance of OTM and Rrobot CAMERA
    :param z: height at wich OTM will be grabbed
    :return:
    """
    roll=roll+90
    x=math.sin(math.radians(roll))*dist+0.08
    y=math.cos(math.radians(roll))*dist
    target_pose=get_quaternion_pose(x,y,z,pitch)
    take_pose(target_pose)
    open_gripper()
    move_home()

def get_quaternion_pose(x,y,z,pitch):
    roll = 0
    signum_helper = np.sign(x + 0.080001) * np.sign(y)
    yaw = np.arctan(np.abs(y) / (np.abs(x) + 0.080001))
    if signum_helper != 0:
        yaw = yaw * signum_helper
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    target_pose = [x, y, z, qx, qy, qz, qw]
    return target_pose

def open_gripper():
    gripper_group_variable_values[0] = 00.010
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)


def close_gripper():
    gripper_group_variable_values[0] = -00.0006
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

def set_gripper_by_value(gripper_value):
    print ("Moving Gripper...")
    gripper_group_variable_values[0] = gripper_value
    gripper_group.set_joint_value_target(gripper_group_variable_values)
    plan2 = gripper_group.go()
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    rospy.sleep(1)

def move_home():
    arm_group.set_named_target("home")
    print ("Executing Move: Home")
    plan1 = arm_group.plan()
    arm_group.execute(plan1, wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    variable = arm_group.get_current_pose()
    print (variable.pose)
    rospy.sleep(1)


def move_position1():
    arm_group.set_named_target("position1")
    print ("Executing Move: Position1")
    plan1 = arm_group.plan()
    arm_group.execute(plan1, wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    variable = arm_group.get_current_pose()
    rospy.sleep(1)
    time.sleep(2)
    print (variable.pose)


def take_pose(target_pose):
    # target pose is [x,y,z,qx,qy,qz]
    print("executing move to target_pose")
    print('moving to', target_pose)
    arm_group.set_pose_target(target_pose)
    plan1 = arm_group.plan()
    arm_group.execute(plan1, wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    variable = arm_group.get_current_pose()
    rospy.sleep(1)

def setup_manipulator():
    print("\nStarting manipulator setup")
    global robot
    global scene
    global arm_group
    global display_trajectory_publisher
    global gripper_group_variable_values
    global gripper_group
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=1)

    # Had probelms with planner failing, Using this planner now. I believe default is OMPL
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    # Increased available planning time from 5 to 10 seconds
    arm_group.set_planning_time(10);

    gripper_group_variable_values = gripper_group.get_current_joint_values()
    print("Manipulation setup has been completed succesfully\n")

if __name__ == '__main__':
    setup_manipulator()