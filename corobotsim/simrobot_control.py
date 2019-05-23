#This modules manages the movements of the simulated robot
#through MoveIt and the Gripper Service.

from os import system
from os import popen
from pose_utils import poseUtilies
from math import pi
from std_msgs.msg import UInt8

class simRobot:
    def __init__(self, rospy_instance, move_group, simulation=False):
        self.rospy          = rospy_instance
        self.move_group     = move_group
        self.ur5_joints     = {'base':0,'shoulder':1,'elbow':2,'wrist_1':3,'wrist_2':4,'wrist_3':5}
        self.USING_GAZEBO   = simulation
        #These offset should be measured and set as precisely as possible
        if self.USING_GAZEBO:
            self.poseUtils      = poseUtilies(self.rospy,self.move_group, WORLD_TO_BASE=(0,0,-1.3), FLANGE_TO_FINGERTIP=0.2)
        else:
            #The distance between flange and fingertip is actually 0.2 but it seems that the URDF model is shorter and thus
            #and additionnal 0.09 is added to get the desired real-world behavior.
            self.poseUtils      = poseUtilies(self.rospy,self.move_group, WORLD_TO_BASE=(0,0,-0.78), FLANGE_TO_FINGERTIP=0.29)
        

    #Stops any residual movement from MoveIt
    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    #Needs to be done before any other robot movement
    #
    # The UR5 arm is effort controlled and has no brakes. 
    # Therefore, at the beginning, the links of the arm are
    # only governed by gravity and joint limits.
    def homingSimulated(self):
        #We place the UR5 in its home position by first moving the elbow
        # and then the other joints. If we did it all at once, we would crash
        # the gripper against the table

        #1) Move the elbow first
        joint_values = self.move_group.get_current_joint_values()
        joint_values[2] = -pi/2.5
        res = self.move_group.go(joint_values, wait=True)
        print('Execution returned:'+str(res))
        #Make sure that there is no residual movements
        self.stop()
        self.rospy.sleep(2)

        #2) Move the arm into its home position
        joint_values = self.move_group.get_current_joint_values()
        joint_values[0] = 0
        joint_values[1] = -pi/2.5
        joint_values[2] = pi/1.5
        joint_values[3] = -pi/1.3
        joint_values[4] = -pi/2
        joint_values[5] = 0
        res = self.move_group.go(joint_values, wait=True)
        print('Execution returned:'+str(res))
        #Make sure that there is no residual movements
        self.stop()

    #Initialization of the robot position by using joint values in RADIANS
    def homing(self, values=None):
        joint_values = self.move_group.get_current_joint_values()
        joint_values[self.ur5_joints['base']]       = values[self.ur5_joints['base']]
        joint_values[self.ur5_joints['shoulder']]   = values[self.ur5_joints['shoulder']]
        joint_values[self.ur5_joints['elbow']]      = values[self.ur5_joints['elbow']]
        joint_values[self.ur5_joints['wrist_1']]    = values[self.ur5_joints['wrist_1']]
        joint_values[self.ur5_joints['wrist_2']]    = values[self.ur5_joints['wrist_2']]
        joint_values[self.ur5_joints['wrist_3']]    = values[self.ur5_joints['wrist_3']]

        res = self.move_group.go(joint_values, wait=True)
        print('Execution returned:'+str(res))

    # Takes a positional parameter where (for 2f-85)
    # 0   = fully opened
    # 180 = fully closed
    # and asynchronously sets the gripper opening/closing
    def set_gripper_closing(self, position):
        command = "rosservice call /gripper/set_position "+str(int(position))
        system(command)

    #Command the gripper's opening relative to current opening
    def set_relative_gripper_closing(self, position):
        #Get current position from /gripper/current_position
        #This does not involve subscribing to a topic
        #Dont ask for a stream if you only need a single drop
        command = "rostopic echo -n 1 /gripper/current_position | head -n 1 | sed 's/data: //'"
        current_position = int(popen(command).read())

        #Add the relative position to current's
        goal_position = current_position + position
        
        #Set the corresponding position
        self.set_gripper_closing(goal_position)
        
    # Move the robot such that the tool reach the pose defined by goal_pos and goal_orient
    # If sync = False, returns before movement is finished.
    def go(self, goal_pos, goal_orient, sync=True, endStop=False):
        #Define the goal pose to reach with the end effector
        pose_goal = self.poseUtils.makeRelToolPose(goal_pos, goal_orient)
        self.move_group.set_pose_target(pose_goal)

        #Return a motion plan (a RobotTrajectory) to the set goal state
        plan = self.move_group.plan()
        
        #Execute the plan if one was found
        res = self.move_group.execute(plan,wait=sync)
        print('Execution returned:'+str(res))
        
        #Make sure that there is no residual movements
        if endStop == True:
            self.stop()

    #Move the end effector relative to its current pose
    def goRel(self,goal_pos_rel=(0,0,0), goal_orient_rel=(0,0,0), sync=True, endStop=False):
        #For implementation details, see self.go()
        pose_goal = self.poseUtils.makeIncrementalEEPose(goal_pos_rel, goal_orient_rel)
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.plan()
        res = self.move_group.execute(plan,wait=sync)
        print('Execution returned:'+str(res))
        if endStop == True:
            self.stop()