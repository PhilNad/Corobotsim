#This module provides a set of utilities to manage
#poses, frames and transformation matrices

from math import pi,radians,degrees
#The following import represents this
#https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
from tf.transformations import *
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import GetModelState

class poseUtilies:
    def __init__(self, rospy_instance, move_group, WORLD_TO_BASE=(0,0,0), FLANGE_TO_FINGERTIP=0.195):
        self.rospy = rospy_instance
        self.move_group = move_group
        #To be manually measured on the robot, see makeRelToolPose()
        self.FLANGE_TO_FINGERTIP = FLANGE_TO_FINGERTIP
        self.WORLD_TO_BASE       = WORLD_TO_BASE
    
    #Create a pose relative to the end-effector current pose given
    # a position difference XYZ and an orientation Euler XYZ angle difference 
    def makeIncrementalEEPose(self, position=(0,0,0), orientation=(0,0,0)):
        pose = self.move_group.get_current_pose().pose

        #We need to translate the orientation in Euler angles first
        orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orient_list)

        #Then we can add it up with the given Euler angles increments
        quat = quaternion_from_euler(roll+radians(orientation[0]), pitch+radians(orientation[1]), yaw+radians(orientation[2]))

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x += position[0]
        pose.position.y += position[1]
        pose.position.z += position[2]
        return pose


    #Create a pose given a XYZ position and Euler XYZ angles orientation
    def makePose(self, position=(0.4,0,0.1), orientation=(0,0,0)):
        pose = Pose()
        quat = quaternion_from_euler(radians(orientation[0]), radians(orientation[1]), radians(orientation[2]))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        return pose

    #Gives the model's pose relative to a link or to the world if the link is empty
    def getModelPose(self, modelName, relativeToLinkname=''):
        pose = None
        try:
            model_coordinates = self.rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(modelName, relativeToLinkname)
            pose = resp_coordinates.pose
        except self.rospy.ServiceException as e:
            self.rospy.loginfo("Get Model State service call failed:  {0}".format(e))
        return pose 

    #Describe a goal pose to reach with our end_effector
    #The position is described in an euclidian form (x,y,z)
    #The orientation is described as a quaternion (x,y,z,w)
    #https://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    def printPose(self, pose):
        orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orient_list)
        roll    = degrees(roll)
        pitch   = degrees(pitch)
        yaw     = degrees(yaw)
        
        print(pose)
        print("Euler Orientation:")
        print("\tRoll  X: "+str(round(roll,1))+" DEG")
        print("\tPitch Y: "+str(round(pitch,1))+" DEG")
        print("\tYaw   Z: "+str(round(yaw,1))+" DEG")

    def printCurrentPose(self):
        print("The EE link is: " + self.move_group.get_end_effector_link())
        print("The Planning frame is: " + self.move_group.get_planning_frame())
        print("Current pose:")
        current_pose = self.move_group.get_current_pose().pose
        self.printPose(current_pose)

    #Defines a pose to use to command the UR5 arm such that the gripper's
    # pose relative to the world reference is define by the arguments.
    #The goal_orient argument is defined by angles in degrees (A,B,C) such that
    #the following steps are followed:
    # 1) Translate the world reference following goal_pos
    # 2) Rotate A degrees around the X axis
    # 3) Rotate B degrees around the new Y axis
    # 4) Rotate C degrees around the newer Z axis
    def makeRelToolPose(self, goal_pos=(0.4,0,1.1), goal_orient=(90,0,90)):
        #Position to reach with the gripper with respect to the world
        goal_position       = goal_pos
        #Orientation relative to world: H = rotx(A)*roty(B)*rotz(C)
        goal_orientation    = (radians(goal_orient[0]),radians(goal_orient[1]),radians(goal_orient[2]))

        #World frame relative to UR5's base frame 
        # AKA steps to bring the base frame onto the world frame using base frame as reference
        Hwb     = compose_matrix(translate=self.WORLD_TO_BASE)
        
        #Gripper frame relative to world frame
        #This defines a transformation matrix:
        # 1) Translate following vector V
        # 2) Rotate A degrees around the X axis
        # 3) Rotate B degrees around the new Y axis
        # 4) Rotate C degrees around the newer Z axis
        #Equivalent formula: H = tr(V)*rotx(A)*roty(B)*rotz(C)
        Hgw     = compose_matrix(translate=goal_pos).dot(euler_matrix(radians(goal_orient[0]), radians(goal_orient[1]), radians(goal_orient[2]), 'rxyz'))
        
        #Wrist frame relative to gripper frame
        Hjg     = compose_matrix(translate=(0,0,-self.FLANGE_TO_FINGERTIP)).dot(euler_matrix(radians(90), radians(-90), 0, 'rxyz'))
        
        #Wrist frame relative to base is Hjb = Hwb*Hgw*Hjg
        Hjb     = Hwb.dot(Hgw).dot(Hjg)

        
        #Extraction of Euler's angles and translation to command the arm
        scale, shear, angles, trans, persp = decompose_matrix(Hjb)
        pose_goal = self.makePose(position=(trans[0],trans[1],trans[2]), orientation=(degrees(angles[0]),degrees(angles[1]),degrees(angles[2])))
        return pose_goal

