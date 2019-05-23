# This modules manages the state of the simulation

from std_srvs.srv import Empty
from std_msgs.msg import Header
from gazebo_msgs.srv import SetModelConfiguration
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

class simManager:
    #Constructor needs to receive a properly initialized rospy instance
    def __init__(self, rospy_instance, move_group, simRobot):
        self.rospy      = rospy_instance
        self.move_group = move_group
        self.simRobot   = simRobot
    
    #Pause the physics engine within the simulation
    #Corresponds to pressing the Play/Pause button in the GUI
    def pausePhysics(self):
        self.rospy.wait_for_service("/gazebo/pause_physics", timeout=1)
        pause = self.rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        pause()
        
    #Unpause the physics engine within the simulation
    #Corresponds to pressing the Play/Pause button in the GUI
    def unpausePhysics(self):
        self.rospy.wait_for_service("/gazebo/unpause_physics", timeout=1)
        unpause = self.rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        unpause()

    #Reset the position of all objects in the world
    def resetWorld(self):
        self.rospy.wait_for_service("/gazebo/reset_world", timeout=1)
        resetObjs = self.rospy.ServiceProxy("/gazebo/reset_world", Empty)
        resetObjs()

    #Reset the robot state to the initial state
    #DOES NOT WORK, work in progress
    def setInitState(self):
        #This UR5 configuration corresponds to makeRelToolPose(goal_pos=(0.4,0.1,1.15), goal_orient=(90,0,90))
        names        = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        positions    = [1.798991006755836, -0.5565362095219593, 0.42343795140628604, -1.2350940061066096, -2.718235869123541, 0.00156069251509372]
        efforts      = [-5.623666590693509, -45.75583256981657, -0.0009394297664042095, 0.5664154774663301, 0.009129034897339586, 0.00017957266874718547]

        #Define a set_joint function that can be used to reset the robots joints
        set_joint   = self.rospy.ServiceProxy("/gazebo/set_model_configuration", SetModelConfiguration)

        try:
            #Arguments of set_joint function
            #['model_name', 'urdf_param_name', 'joint_names', 'joint_positions']
            res = set_joint('coro','',names, positions)
            if not res.success:
                self.rospy.logwarn(res.status_message)
        except self.rospy.ServiceException as e:
            self.rospy.logerr("Service call failed: %s" % e)

        #Update MoveIt! robot state so it syncs with gazebo's
        joint_state                     = JointState()
        joint_state.header              = Header()
        joint_state.header.stamp        = self.rospy.Time.now()
        joint_state.name                = names
        joint_state.position            = positions
        moveit_robot_state              = RobotState()
        moveit_robot_state.joint_state  = joint_state
        self.move_group.set_start_state_to_current_state()
        #This line crashes the simulation
        #self.move_group.set_start_state(moveit_robot_state)

    #Stops any residual movement from MoveIt
    def stop(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    #Resets the robot position
    def reset(self):
        #This does not work, because the robot is effort controller
        #and the controller is not aware of the change
        #self.setInitState()

        #For debugging
        #state = self.rospy.wait_for_message("/joint_states", JointState, timeout=1.0)
        #print(state)

        #Go to the starting pose
        self.simRobot.go(goal_pos=(0.4,0.1,1.15), goal_orient=(90,0,90), sync=True)
        #Resets objects position
        self.resetWorld()