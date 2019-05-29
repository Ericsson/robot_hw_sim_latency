import gym
import gym_gazebo
import rospy
import roslaunch
import rosnode
from roslaunch.core import Node, Master, RLException
from rospkg import RosPack
rp = RosPack()
import os
import numpy as np
import signal
import subprocess
import time
from os import path
from std_srvs.srv import Empty
from std_msgs.msg import String
import random
from threading import Event

class Ariac2017LatencyEnv(gym.Env):
    """Ariac 2017 latency env
    """
    metadata = {'render.modes': ['human']}
    
    def on_shutdown(self):
        print("node shutdown request")
        self.shutdown=True
        self.next_step_event.set()
    
    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            rospy.loginfo("Changed comp state: " + str(msg.data))
        self.current_comp_state = msg.data
        if msg.data == 'done': self.next_step_event.set()
        
        
    def action_callback(self, msg):
        if self.current_action_state != msg.data:
            rospy.loginfo("Changed action state: " + str(msg.data))
            self.current_action_state = msg.data
            self.next_step_event.set()
       
        

    def __init__(self):

        # allapotter 2 elemu legyen, High vagy Low QoC allapot kozott valt
        self.action_space = gym.spaces.Discrete(2) #High, Low
        self.reward_range = (-np.inf, np.inf)
        # a figment altal kiadott fv, pl. go_to_belt, az jo state lesz nekunk
        
        self.osrf_gear_folder = rp.get_path('osrf_gear')
        self.figment_folder = rp.get_path('figment_ariac')
        self.ur5_vel_start_folder = rp.get_path('ur5_vel_start')
        
        figment_actions =  ['start',
                            'go_to_initial_position', 
                            'go_to_tray_position', 
                            'go_to_belt_start', 
                            'go_to_belt', 
                            'go_to_part_bin_front', 
                            'go_to_bin_front', 
                            'go_to_position_a_bit_above_part', 
                            'go_down_until_get_piece', 
                            'turnWrist', 
                            'MoveSideWays', 
                            'move_towards_piece_on_belt_part1',
                            'move_towards_piece_on_belt_part2',
                            'move_towards_piece_on_belt_part3',
                            'moveToolTip', 
                            'moveToolTipZY',
                            'discard_part_from_belt', 
                            'go_discard_from_tray1', 
                            'go_to_discard_open_bin', 
                            'turn_pulley_yellow_bar']   
        
        self.figment_action_to_index = dict([(a,i) for i, a in enumerate(figment_actions)])
        self.observation_space = gym.spaces.Discrete(len(figment_actions))
        #spaces.Tuple((spaces.Discrete(2), spaces.Discrete(3)))
        #print(self.osrf_gear_folder)
        #print(self.figment_folder)
        #
        self.next_step_event = Event()
        self.current_comp_state = None
        self.prev_comp_state = None
        self.current_action_state = "start"
        self.figment_process = None
        self.ariac_launch = None

        self.started = False
        self.shutdown = False


    def stop_all(self):
        if self.shutdown:
            print("Already stopping")
            return
        if self.started:
            print("Stopping all!")
            
            self.started = False
            self.shutdown = True
            self.comp_state_sub.unregister()
            self.figment_action_sub.unregister()
            self.latency_pub.unregister()
            self.comp_state_sub.unregister()
            rospy.signal_shutdown("Stopping all!")
            if self.figment_process is not None:
                self.figment_process.stop()
                self.figment_process = None
            if self.ariac_launch is not None:
                self.ariac_launch.shutdown()
                self.ariac_launch = None
            time.sleep(20)
        else:
            print("Stopped but not started")

    
    def start_all(self):
        self.started = True
        self.shutdown = False
        self.current_comp_state = None
        self.prev_comp_state = None
        self.current_action_state = "start"
        self.figment_process = None
        self.ariac_launch = None
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        
        roslaunch_file = [(self.osrf_gear_folder+"/launch/figment_environment.launch", ["competition:=finals/final01.yaml",])]        
        self.ariac_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file , is_core=True, verbose=True)
        print("Created ariac ROSLaunch")
        self.ariac_launch.start(auto_terminate=True)
        print("Started ariac roslaunch")
        
        rospy.init_node('gym', anonymous=False, disable_signals=True)
        #rospy.on_shutdown(self.on_shutdown)
        
        self.comp_state_sub = rospy.Subscriber("/ariac/competition_state", String, self.comp_state_callback)
        self.figment_action_sub = rospy.Subscriber("/figment/action", String, self.action_callback)
        self.latency_pub = rospy.Publisher('/latency_plugin_simple_queue/priority', String, queue_size=1)
        self.comp_state_sub = rospy.Subscriber("/ariac/competition_state", String, self.comp_state_callback)
        
        rospy.wait_for_service('/gazebo/unpause_physics',5)
        
        figment_node = roslaunch.core.Node("figment_ariac", "scheduler_plan", 
            output="screen")
        print("Created figment node")
        
        self.figment_process, success = self.ariac_launch.runner.launch_node(figment_node)
        print("Launched figment node success:"+str(success))
        time.sleep(5)
        
        #Step simulation 4 times to start the controllers
        subprocess.call(["gz","world","-m","4"])
        
        time.sleep(10)
        
        # Ursim 
        # HARDCODED PATHS!!!!!!!!!!
        #ursim_node = roslaunch.core.Node("ur5_vel_start", "run_in_network_namespace_with_latency.py", 
        #    args='"sudo -u user /home/user/ursim-3.5.3.10825/start-ursim.sh"', 
        #    output="screen")
        #print("Created ursim node")
        
        #ariac_launch.runner.launch_node(ursim_node)
        #print("Launched ursim node")
        
        #time.sleep(30)
        
        #~ uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #~ roslaunch.configure_logging(uuid)
        
        #~ roslaunch_file = [(self.ur5_vel_start_folder+"/launch/setup.launch", ["robot_ip:=10.0.1.2", "sim:=false", "ns:=realur/", "moveit:=false"])]        
        #~ urcontroller_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file , is_core=False, verbose=True)
        #~ print("Created ur5_vel_start ROSLaunch")
        #~ urcontroller_launch.start(auto_terminate=False)
        #~ print("Started ur5_vel_start roslaunch")
        
        #~ time.sleep(5)
        

    
        
        
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        print("STARTED run!")

        #self.unpause()


        #uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)
        
        
        #roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
                
        #launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        #launch.start()
        
        
        


    def step(self, action):
        done = False
        extra_data = {}
        reward = 0
        
        rospy.wait_for_service('/gazebo/unpause_physics',5)
        self.latency_pub.publish('High' if action == 0 else 'Low')
        self.unpause()
        rospy.loginfo("Waiting for next step event")
        flag = None
        for name in rosnode.get_node_names():
            print(name)
        try:
            nodes = rosnode.get_node_names()
        except Exception as ex:
            print("Caught ex:")
            print(str(ex))
            nodes=[]
            pass
        while len(nodes) > 2:
            flag = self.next_step_event.wait(2)
            if flag:
                break
        if not flag: 
            done = True 
            extra_data["sudden_exit"]=True
        
        if self.current_comp_state == 'done':
            done = True
        if not done:
            self.pause()
        
        obs = self.figment_action_to_index[self.current_action_state]
        self.next_step_event.clear()
        return obs, reward, done, extra_data

        # Implement this method in every subclass
        # Perform a step in gazebo. E.g. move the robot

        #rospy.wait_for_service('/gazebo/unpause_physics')
        #try:
            #self.unpause()
        #except (rospy.ServiceException) as e:
            #print ("/gazebo/unpause_physics service call failed")

        ## a valtast a script csinalja a tc delay-ek atallitasaval
        #if action == 0: #HIGH
        ##system.call('high')
        #elif action == 1: #LOW
        ##system.call('low')

        #data = None
        ## ha kap egy ariac score-t akkor az episode done-nak minosul
        #while data is None:
            #try:
        ## ariac scoring alapjan reward oriasi sullyal *1000
                #data = rospy.wait_for_message('/ariac_score', timeout=50)
            #except:
                #pass

        #rospy.wait_for_service('/gazebo/pause_physics')
        #try:
            ##resp_pause = pause.call()
            #self.pause()
        #except (rospy.ServiceException) as e:
            #print ("/gazebo/pause_physics service call failed")

        #state,done = self.discretize_observation(data,5)

        ## minden step utan legyen reward
        ## az alapjan h high vagy low QOC-ben volt, low-nal kap tobbet steppenkent
        ## vegrehajtasi ido egy egy fv-nek, a kicsi jobb
        ## desired es current position error, kicsi jobb
        #if not done:
        ## nezni kellene az allapotat a gazebo-nak
        ## ha beakadt, nekiment vminek, tul nagy az error a gazebo sim es a valos robot kozott akkor az episode not done

        
        ##ha nem akadt be akkor vmi action-t valasszon es jo lenne ha lenne koztes kicsi reward is, pl. kozepen fogta meg
            #if action == 0:
                #reward = 5
            #else:
                #reward = 1
        #else:
            #reward = -200

        #return state, reward, done, {}


        #raise NotImplementedError
        
    def reset(self):
        if self.started: 
            self.stop_all()
         
            self.started = False
            self.shutdown = False
        self.start_all()
        return self.figment_action_to_index['start']



    def render(self, mode="human"):
        return

    def close(self):
        self.stop_all()

    #~ def _configure(self):

        #~ # TODO
        #~ # From OpenAI API: Provides runtime configuration to the enviroment
        #~ # Maybe set the Real Time Factor?
        #~ pass
    def seed(self):

        # TODO
        # From OpenAI API: Sets the seed for this env's random number generator(s)
        pass
