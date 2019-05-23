#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi
from copy import deepcopy

#Corobotsim's modules
from scene_manager import sceneManager
from simulation_manager import simManager
from simrobot_control import simRobot

#For spawnTestCube
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

#Set to True if you are using Gazebo, false if you are using the real UR5.
USING_GAZEBO = True

def spawnTestCube():
    path = '../cube.sdf'
    with open(path, 'r') as basefile:
        model_xml=basefile.read()
    base = deepcopy(model_xml)

    #Spawn the insertion base
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    req = SpawnModelRequest()
    req.model_name  = "Cube"
    req.model_xml   = base
    req.initial_pose = Pose()
    req.initial_pose.position.x = 0.5
    req.initial_pose.position.y = 0
    req.initial_pose.position.z = 1.01
    resp = spawn_model(req)
    rospy.sleep(1)


if __name__ == "__main__":
    #Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('coro_test', anonymous=True)
    
    #The "arm" planning group as described in coro.srdf represents the UR5
    move_group  = moveit_commander.MoveGroupCommander("arm")

    #Manages the objects in the scene
    sceneMan    = sceneManager(rospy, moveit_commander)

    #Manages robot's movements
    simRob      = simRobot(rospy, move_group, simulation=USING_GAZEBO)

    if USING_GAZEBO == True:
        #Manages the state of the Gazebo simulation
        simMan      = simManager(rospy, move_group, simRob)
    
    #Add a box that represents the table so MoveIt is aware of its presence
    # and hopefully avoids collisions with it. Needs to be done while the arm
    # does not touch the table.
    if USING_GAZEBO == True:
        table_position  = (0,   0,   -0.8)
        table_size      = (1.5, 0.8, 1)
        sceneMan.addBoxToMoveit("table",table_position, table_size)
    else:
        table_position  = (0, 0.5, -0.78)
        table_size      = (1, 1,    0.1)
        sceneMan.addBoxToMoveit("table",table_position, table_size)
        wall_position   = (0, 0,   -0.65)
        wall_size       = (1, 0.1,  1)
        sceneMan.addBoxToMoveit("wall",wall_position, wall_size)

    #Initialization of the robot position
    if USING_GAZEBO == True:
        simRob.homingSimulated()
    else:
        joint_values = [0 for i in simRob.ur5_joints]
        joint_values[simRob.ur5_joints['base']]        = 1.1657141276613643
        joint_values[simRob.ur5_joints['shoulder']]    = 0.151321586314704
        joint_values[simRob.ur5_joints['elbow']]       = 1.3452886426420623
        joint_values[simRob.ur5_joints['wrist_1']]     = 3.135452811588985
        joint_values[simRob.ur5_joints['wrist_2']]     = -1.532097634506619
        joint_values[simRob.ur5_joints['wrist_3']]     = 1.5829452509745248
        simRob.homing(joint_values)

    #Reset of the objects of the simulated scene
    if USING_GAZEBO == True:
        simMan.reset()
        #Spawn the insertion base
        sceneMan.spawnInsertionBase()
        #Spawn the square rod
        #sceneMan.spawnInsertionSquareRod()
	sceneMan.spawnInsertionRoundRod()
	#Below code is for the spawnTestCube() experiment
	#spawnTestCube()
    
    #Open the gripper
    simRob.set_gripper_closing(0)
    #Go near the rod but a bit behind so we dont tilt it over
    simRob.go(goal_pos=(0.41125,0.1,1.15), goal_orient=(90,0,90), sync=True)

    #Approach the rod
    simRob.goRel(goal_pos_rel=(0,-0.065,0))

    #Closes the gripper over the rod
    simRob.set_relative_gripper_closing(170)
    #Wait to make sure the movement is done
    rospy.sleep(1)

    #Lift the rod
    simRob.goRel(goal_pos_rel=(0,0,0.1))

    #Put the round on top of the round slot
    #X:(0.2+0.114909)-(0.41125)=-0.0963
    #Y:-(0.2+0.05)-(0.1-0.065)=-0.285
    simRob.goRel(goal_pos_rel=(-0.096,-0.162,0))

    #Put the rod down and release it
    simRob.goRel(goal_pos_rel=(0,0,-0.05))
    simRob.set_gripper_closing(0)
    rospy.sleep(1)

    #Go back to our retreated position
    simRob.go(goal_pos=(0.41125,0.1,1.15), goal_orient=(90,0,90), sync=True)

    #Below code is for spawnTestCube() friction tests
    #simRob.goRel(goal_pos_rel=(0,-0.108,0))
    #simRob.goRel(goal_pos_rel=(0,0,-0.18))

    #simRob.set_gripper_closing(180)
    #rospy.sleep(1)
    #simRob.goRel(goal_pos_rel=(0,0,0.18))
