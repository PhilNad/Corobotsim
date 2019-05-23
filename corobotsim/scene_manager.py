# This modules manages the simulated Gazebo scene
# by adding, removing or modifying objects

from copy import deepcopy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class sceneManager:
    #Constructor needs to receive a properly initialized rospy instance
    def __init__(self, rospy_instance, moveit_commander):
        self.rospy = rospy_instance
        self.moveit_robotCmd = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        #DO NOT REMOVE this delay, its needed to let the PlanningSceneInterface initialize.
        self.rospy.sleep(1)

        #Directory where all the .dae, .stl and .sdf files are kept
        #relative to this module
        self.BASE_PATH = '../InsertionTaskFiles/'
            

    #Add a collision box around the table so MoveIt finds
    #a way around it.
    def addBoxToMoveit(self, name, position=(0,0,-0.7), size=(1.5, 0.8, 1)):
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.moveit_robotCmd.get_planning_frame()
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]

        self.moveit_scene.add_box(name, box_pose, size=size)
        #DO NOT REMOVE this delay, its needed 
        # to let the PlanningSceneInterface update.
        self.rospy.sleep(1)

        print("Objects added to MoveIt planning scene:")
        print(self.moveit_scene.get_known_object_names())

    #Spawn the base in which we will insert the rods
    def spawnInsertionBase(self):
        #Open the original SDF file
        path = self.BASE_PATH + 'Base.sdf'
        with open(path, 'r') as basefile:
            model_xml=basefile.read()
        base = deepcopy(model_xml)

        #Physics constants as calculated with Meshlab
        scale   = 1e-3
        volume  = 1402889 
        density = 8000 #Steel
        com     = (149.659271, 50, 25)
        inertia = (1537954048, 596, 298, 11058070528, -16, 12011485184)
        #Modify the SDF to use these physical properties
        base = self.modifySDF(base, scale, density, volume, com, inertia)

        #Spawn the insertion base
        spawn_model = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name  = "InsertionBase"
        req.model_xml   = base
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.2
        req.initial_pose.position.y = -0.2
        req.initial_pose.position.z = 1.01
        resp = spawn_model(req)
        self.rospy.sleep(1)

    #Spawn the round rod
    def spawnInsertionRoundRod(self):
        #Open the original SDF file
        path = self.BASE_PATH + 'RoundRod.sdf'
        with open(path, 'r') as basefile:
            model_xml=basefile.read()
        sdf = deepcopy(model_xml)

        #Physics constants as calculated with Meshlab
        scale   = 1e-3
        volume  = 97677 
        density = 8000 #Steel
        com     = (12.5, 12.5, 100)
        inertia = (329386560, 0, -5, 329386560, -5, 7592381)
        #Modify the SDF to use these physical properties
        sdf = self.modifySDF(sdf, scale, density, volume, com, inertia)

        #Spawn the insertion base
        spawn_model = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name  = "RoundRod"
        req.model_xml   = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.y = 0
        req.initial_pose.position.z = 1.01
        resp = spawn_model(req)
        self.rospy.sleep(1)

    #Spawn the square rod
    def spawnInsertionSquareRod(self):
        #Open the original SDF file
        path = self.BASE_PATH + 'SquareRod.sdf'
        with open(path, 'r') as basefile:
            model_xml=basefile.read()
        sdf = deepcopy(model_xml)

        #Physics constants as calculated with Meshlab
        scale   = 1e-3
        volume  = 80000 
        density = 500 #Steel = 8000
        com     = (10, 10, 100)
        #inertia = (269333376, 0, 0, 269333376, 0, 5333334)
        inertia = (269333376, 0, 0, 269333376, 0, 269333376)

        #Modify the SDF to use these physical properties
        sdf = self.modifySDF(sdf, scale, density, volume, com, inertia)

        #Spawn the insertion base
        spawn_model = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name  = "SquareRod"
        req.model_xml   = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.y = 0
        req.initial_pose.position.z = 1.01
        resp = spawn_model(req)
        self.rospy.sleep(1)

    #Spawn the hexagonal rod
    def spawnInsertionHexagonalRod(self):
        #Open the original SDF file
        path = self.BASE_PATH + 'HexRod.sdf'
        with open(path, 'r') as basefile:
            model_xml=basefile.read()
        sdf = deepcopy(model_xml)

        #Physics constants as calculated with Meshlab
        scale   = 1e-3
        volume  = 108196 
        density = 8000 #Steel
        com     = (12.496746, 14.429998, 100)
        inertia = (365349728, -3, 0, 365349600, 10, 9387199)
        #Modify the SDF to use these physical properties
        sdf = self.modifySDF(sdf, scale, density, volume, com, inertia)

        #Spawn the insertion base
        spawn_model = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name  = "HexRod"
        req.model_xml   = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.y = 0
        req.initial_pose.position.z = 1.01
        resp = spawn_model(req)
        self.rospy.sleep(1)

    #Spawn the slot rod
    def spawnInsertionSlotRod(self):
        #Open the original SDF file
        path = self.BASE_PATH + 'SlotRod.sdf'
        with open(path, 'r') as basefile:
            model_xml=basefile.read()
        sdf = deepcopy(model_xml)

        #Physics constants as calculated with Meshlab
        scale   = 1e-3
        volume  = 102513 
        density = 8000 #Steel
        com     = (10, 15, 100)
        inertia = (347808608, -0.5, -0.5, 344599360, 5.5, 8985750)
        #Modify the SDF to use these physical properties
        sdf = self.modifySDF(sdf, scale, density, volume, com, inertia)

        #Spawn the insertion base
        spawn_model = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        req = SpawnModelRequest()
        req.model_name  = "SlotRod"
        req.model_xml   = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = 0.4
        req.initial_pose.position.y = 0
        req.initial_pose.position.z = 1.01
        resp = spawn_model(req)
        self.rospy.sleep(1)

    #Modify an SDF file to include geometric physical constants
    #To get a list of available Gazebo materials do: 
    #cat /usr/share/gazebo-9/media/materials/scripts/gazebo.material | grep material
    #   Parameters:
    #       Scale: Number of times the model is scaled up relative to METERS.
    #              If the model is using MM, scale should equal 1e-3
    #       Density: Homogenous density of the material the object is made of.
    #                Should be given in Kg/m^3.
    #       Volume: Volume of the object.
    #       Center of mass: Tuple (X,Y,Z) that gives the COM relative to the origin.
    #       Inertia Tensor: Tuple (IXX, IXY, IXZ, IYY, IYZ, IZZ)
    def modifySDF(self, sdf, scale, density, volume, center_of_mass, inertia_tensor):
        mass = volume * scale**3 * density
        print("Mass of added object: "+str(mass)+"Kg")

        com_x = center_of_mass[0]  * scale
        com_y = center_of_mass[1]  * scale
        com_z = center_of_mass[2]  * scale

        ixx = inertia_tensor[0] * scale**5 * density
        ixy = inertia_tensor[1] * scale**5 * density
        ixz = inertia_tensor[2] * scale**5 * density
        iyy = inertia_tensor[3] * scale**5 * density
        iyz = inertia_tensor[4] * scale**5 * density
        izz = inertia_tensor[5] * scale**5 * density

        sdf = sdf.replace('COM_X', str(com_x))
        sdf = sdf.replace('COM_Y', str(com_y))
        sdf = sdf.replace('COM_Z', str(com_z))
        sdf = sdf.replace('MASS', str(mass))
        sdf = sdf.replace('INERTIA_TENSOR_XX', str(ixx))
        sdf = sdf.replace('INERTIA_TENSOR_XY', str(ixy))
        sdf = sdf.replace('INERTIA_TENSOR_XZ', str(ixz))
        sdf = sdf.replace('INERTIA_TENSOR_YY', str(iyy))
        sdf = sdf.replace('INERTIA_TENSOR_YZ', str(iyz))
        sdf = sdf.replace('INERTIA_TENSOR_ZZ', str(izz))

        return sdf
