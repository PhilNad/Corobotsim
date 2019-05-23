#This module define an OpenAI Gym environment that
#represents the UR-5 + 2f-85 robot used.

import gym

class CoroEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        #Define state space and action space
        pass

    #Once the agent chose which action to take, this function is called
    def step(self, action):
        """

        Parameters
        ----------
        action :

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing your observation of
                the environment.
            reward (float) :
                amount of reward achieved by the previous action. The scale
                varies between environments, but the goal is always to increase
                your total reward.
            episode_over (bool) :
                whether it's time to reset the environment again. Most (but not
                all) tasks are divided up into well-defined episodes, and done
                being True indicates the episode has terminated. (For example,
                perhaps the pole tipped too far, or you lost your last life.)
            info (dict) :
                 diagnostic information useful for debugging. It can sometimes
                 be useful for learning (for example, it might contain the raw
                 probabilities behind the environment's last state change).
                 However, official evaluations of your agent are not allowed to
                 use this for learning.
        """

        #Execute the chosen action
        self.take_action(action)

        #Get the resulting new state
        observation     = self.get_observation()
        
        #Compute the reward
        reward          = self.get_reward()
        
        #Not used
        episode_over    = False
        info = {}

        return observation, reward, episode_over, info

    def get_observation(self):
        #Returns and object representing the state:
        #-Pose of the manipulated object ((x,y,z),(x,y,z))
        #-Taxels of the two tactile sensors ((1,..,28),(1,..,28))
        #-Force/Torque sensor vector ((x,y,z))
        #Dimensions: 6+56+3 = 65

    def reset(self):
        simMan.reset()
        return get_observation()

    def take_action(self, action):
        #This tuple defines the action
        #(pos.X, pos.Y, pos.Z, orient.X, orient.Y, orient.Z, gripper)
        
        posX = action[0]
        posY = action[1]
        posZ = action[2]
        orientX = action[3]
        orientY = action[4]
        orientZ = action[5]
        opening = action[6]

        #Take the action
        simRob.set_relative_gripper_closing(opening)
        simRob.goRel(goal_pos_rel=(posX,posY,posZ), goal_orient_rel=(orientX, orientY, orientZ))
        

    def get_reward(self):
        #Implements reward shaping for the task
        #Requires to quantify the quality of the state we're in
        #so it depends on the goal
        return float()