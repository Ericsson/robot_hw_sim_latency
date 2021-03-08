from __future__ import division
from __future__ import print_function

import os
import socket
import numpy as np
import math
import signal
from threading import Event
import subprocess
import time
import random
from os import path

import gym
import rospy
import roslaunch
import rosnode
from roslaunch.core import Node, Master, RLException
from rospkg import RosPack
rp = RosPack()

from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from osrf_gear.msg import VacuumGripperState

from pyroute2 import IPRoute


def send_to_tcp(hostname, port, content):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((hostname, port))
    s.sendall(content)
    s.shutdown(socket.SHUT_WR)
    s.close()

GREEN = "\033[42;1m"
CLEARC = "\033[0m"
def log(string):
    print(GREEN + string + CLEARC)

def get_result():
    score = ""
    game_score = 0
    process_time = 0
    part_time = 0
    with open(os.environ['HOME']+"/.gazebo/server-11345/default.log") as logfile:
        inscore = False
        for line in logfile:
            if line.startswith("<game_score>"):
                inscore = True
            if inscore:
                score+=line
            if line.startswith("</game_score>"):
                break
    for line in score.splitlines(): 
        parsedline = line.split(': ')
        log(str(parsedline))
        if parsedline[0] == 'Total game score':
            game_score = int(parsedline[1][1:-1])
        if parsedline[0] == "Total process time":
            process_time = float(parsedline[1][1:-1])
        if parsedline[0] == "Part travel time":
            part_time = float(parsedline[1][1:-1])
            
             
    #rospy.logerr(score)
    rospy.logerr("gamescore = "+str(game_score))
    return game_score, process_time, part_time

def get_newest_folder(folder):
    dirs = [os.path.join(folder,d) for d in os.listdir(folder) if os.path.isdir(os.path.join(folder,d))]
    if len(dirs) == 0:
        return folder
    return max(dirs, key=os.path.getmtime)

def write_result(ratio,rs, ttime, ptime):
    with open(get_newest_folder(get_newest_folder("/root/ray_results"))+"/result.txt","a+") as resultfile:
        resultfile.write("LowQoC ratio: " + str(ratio) + "	Gamescore:" + str(rs) +"	Total time:" + str(ttime) + "	Part time:" + str(ptime) +"\n")

class Ariac2017LatencyEnv(gym.Env):
    """Ariac 2017 latency env
    """
    metadata = {'render.modes': ['human']}
    
    def on_shutdown(self):
        log("node shutdown request")
        self.stop_all()
        #self.next_step_event.set()
    
    def comp_state_callback(self, msg):
        if self.current_comp_state != msg.data:
            log("Changed comp state: " + str(msg.data))
        self.current_comp_state = msg.data
        #if msg.data == 'done': self.next_step_event.set()

    def game_score_callback(self, msg):
        if self.current_game_score != msg.data:
            rospy.loginfo("Game score message: " + str(msg.data))
        self.current_game_score = msg.data
        
    def action_callback(self, msg):
        if self.current_action_state != msg.data:
            rospy.loginfo("Changed action state: " + str(msg.data))
            self.current_action_state = msg.data
            #self.next_step_event.set()

    def joint_state_callback(self, msg):
        #print("CALLBACK BEGIN")
        #rospy.loginfo("Joint State Data: \n " + str(msg))
        self.current_joint_state = msg
        #print("CALLBACK END")

    def gripper_state_callback(self, msg):
        self.current_gripper_state = msg
        #print("CALLBACK VLUE: ", msg)
        
    def get_obs(self):
        pos = np.array(self.current_joint_state.position)[self.permutation]
        vel = np.array(self.current_joint_state.velocity)[self.permutation]
        eff = np.array(self.current_joint_state.effort)[self.permutation]

        obs = (1 if self.current_gripper_state.attached == True else 0, 
            np.clip(pos,-self.posHigh+0.01,self.posHigh-0.01), 
            np.clip(vel,-self.veloHigh,self.veloHigh), 
            np.clip(eff,-self.effortHigh,self.effortHigh)
        )
        return obs

    def __init__(self, digital_twin=False, robot_ip="0.0.0.0"):
        self.digital_twin = digital_twin
        self.robot_ip = robot_ip

        self.posHigh = np.array(8*[2*(math.pi)+0.01])
        self.veloHigh = np.array(8*[20])
        self.effortHigh = np.array(8*[1000])
        # allapotter 2 elemu legyen, High vagy Low QoC allapot kozott valt
        self.action_space = gym.spaces.Discrete(2) #High, Low
        self.reward_range = (-np.inf, np.inf)
        self.observation_space = gym.spaces.Tuple((
            gym.spaces.Discrete(2),
            gym.spaces.Box(-self.posHigh,self.posHigh),
            gym.spaces.Box(-self.veloHigh,self.veloHigh),
            gym.spaces.Box(-self.effortHigh,self.effortHigh)
        ))
        log(str(self.observation_space))
        # a figment altal kiadott fv, pl. go_to_belt, az jo state lesz nekunk
        
        self.osrf_gear_folder = rp.get_path('osrf_gear')
        self.figment_folder = rp.get_path('figment_ariac')
        self.ur5_vel_start_folder = rp.get_path('ur5_vel_start')
        self.hAction = self.lAction = 0.0
        self.qReward = 0
        
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
	    
        # there is no scenario 0
        self.max_points_scenario = [0, 6, 9,18,24,24,21,23,18,6,15,24,15,18,24,24]


        #!!! figment action-hoz kell !!!
        #self.next_step_event = Event()
        self.current_comp_state = None
        self.current_game_score = None
        self.prev_game_score = None
        self.current_joint_state = JointState()
        self.current_gripper_state = VacuumGripperState()
        self.prev_comp_state = None
        self.current_action_state = "start"
        self.figment_process = None
        self.ariac_launch = None
        self.urcontroller_launch = None
        
        self.permutation = None

        self.unpause = None
        self.pause = None

        rospy.init_node('gym', anonymous=True, disable_signals=True)
        rospy.on_shutdown(self.on_shutdown)

        self.started = False
        self.shutdown = False
    def stop_all(self):
        if self.shutdown:
            log("Already stopping")
            return
        if self.started:
            log("Stopping all!")
            
            self.started = False
            self.shutdown = True
            
            time.sleep(5)
            
            if self.comp_state_sub is not None:
                self.comp_state_sub.unregister()
            if self.figment_action_sub is not None:
                self.figment_action_sub.unregister()
            if self.joint_state_sub is not None:
                self.joint_state_sub.unregister()
            if self.gripper_state_sub is not None:
                self.gripper_state_sub.unregister()
            if self.latency_pub is not None:
                self.latency_pub.unregister()
            if self.unpause is not None:
                self.unpause.close()
            if self.pause is not None:
                self.pause.close()
            if self.figment_process is not None:
                self.figment_process.stop()
                while self.figment_process.is_alive():
                    log("Waiting for figment stop!")
                    time.sleep(1)
                    
            if self.ariac_launch is not None:
                self.ariac_launch.shutdown()
                while not self.ariac_launch.runner.pm.done:
                    log("Waiting for ariac_launch to stop!")
                    time.sleep(1)
            if self.urcontroller_launch is not None:
                self.urcontroller_launch.shutdown()
                while not self.urcontroller_launch.runner.pm.done:
                    log("Waiting for urcontroller_launch to stop!")
                    time.sleep(1)
                #ariac_launch = None
            try:
                if self.digital_twin and self.tunnel_if:
                    log("Deleting delay on eth1")
                    self.ipr.tc("del", "netem", self.tunnel_if)
            except:
                pass
                
            self.permutation = None
            
            time.sleep(15)
        else:
            log("Stopped but not started")

    
    def start_all(self):
        log("START ALL BEGIN {}".format("DT" if self.digital_twin else ""))
        self.started = True
        self.shutdown = False
        self.current_comp_state = None
        self.current_game_score = 0.0
        self.prev_game_score = 0.0
        self.current_joint_state = JointState()
        self.current_gripper_state = VacuumGripperState()
        self.prev_comp_state = None
        self.current_action_state = "start"
        self.figment_process = None
        self.ariac_launch = None
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.running_scenario = 15#random.randint(1,15) # 2 error?
        self.max_score = self.max_points_scenario[self.running_scenario]
        log("Running scenario "+str(self.running_scenario).zfill(2) + " with max score " + str(self.max_score))
        
        if self.digital_twin:
            roslaunch_file = [(self.ur5_vel_start_folder+"/launch/setup.launch", ["robot_ip:="+self.robot_ip, "sim:=false", "ns:=realur/", "moveit:=false", "latency_plugin:=true"])]        
            self.urcontroller_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file , is_core=False, verbose=True)
            log("Created ur5_vel_start ROSLaunch")
            self.urcontroller_launch.start(auto_terminate=True)
            log("Started ur5_vel_start roslaunch")
            
            # TC init stuff
            self.ipr = IPRoute()
            interfaces = self.ipr.link_lookup(ifname='eth1')
            if len(interfaces) == 1:
                self.tunnel_if = interfaces[0]
                try:
                    log("Creating delay on eth1")
                    self.ipr.tc("add", "netem", self.tunnel_if, delay=1000)
                except:
                    log("Retrying creating delay on eth1")
                    self.ipr.tc("del", "netem", self.tunnel_if)
                    self.ipr.tc("add", "netem", self.tunnel_if, delay=1000)
                    pass
            else:
                self.tunnel_if = None
        
            time.sleep(5)
        
        if not self.digital_twin:
            roslaunch_file = [(self.osrf_gear_folder+"/launch/figment_environment.launch", ["competition:=finals/final"+str(self.running_scenario).zfill(2)+".yaml","digital_twin:=false"])]        
        else:
            roslaunch_file = [(self.osrf_gear_folder+"/launch/figment_environment.launch", ["competition:=finals/final"+str(self.running_scenario).zfill(2)+".yaml","digital_twin:=true"])]  
        self.ariac_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file , is_core=False, verbose=True)
        log("Created ariac ROSLaunch")
        self.ariac_launch.start(auto_terminate=True)
        log("Started ariac roslaunch")
        
        if self.digital_twin:
            init_command = "movej([3.14,-1.13,1.51,3.77,-1.51,0.0])\nend"
            send_to_tcp(self.robot_ip, 30002, init_command)
            log("Sending init command to UR...")
        
        self.comp_state_sub = rospy.Subscriber("/ariac/competition_state", String, self.comp_state_callback)
        self.score_sub = rospy.Subscriber("/ariac/current_score", Float32, self.game_score_callback)
        self.figment_action_sub = rospy.Subscriber("/figment/action", String, self.action_callback)
        self.latency_pub = rospy.Publisher('/latency_plugin_simple_queue/priority', String, queue_size=1)
        self.joint_state_sub = rospy.Subscriber('/ariac/joint_states', JointState, self.joint_state_callback)
        self.gripper_state_sub = rospy.Subscriber('/ariac/gripper/state', VacuumGripperState, self.gripper_state_callback)
        log("SUBSCRIBED")
                
        figment_node = roslaunch.core.Node("figment_ariac", "scheduler_plan", 
            output="screen")
        log("Created figment node")
        
        self.figment_process, success = self.ariac_launch.runner.launch_node(figment_node)
        log("Launched figment node success:"+str(success))
        time.sleep(10)

        #Step simulation 4 times to start the controllers
        subprocess.call(["gz","world","-m","10"])
        time.sleep(2)
        subprocess.call(["gz","world","-m","20"])
        time.sleep(10)
        
        desired_order = np.array(["linear_arm_actuator_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",  
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "vacuum_gripper_joint"])
        
        tries = 0
        while len(self.current_joint_state.name) == 0:
            log("Waiting for JointState")
            subprocess.call(["gz","world","-m","10"])
            tries += 1
            if tries > 3:
                self.stop_all()
                exit(1)
        scrambled = np.array(self.current_joint_state.name)
        # create unscrambler permutation
        self.permutation = np.where(desired_order[:, None] == scrambled[None, :])[1]
        log("UNSCRAMBLER PERMUTATION = " + str(self.permutation))
        
        #time.sleep(20)
        
        # Ursim 
        # HARDCODED PATHS!!!!!!!!!!
        #ursim_node = roslaunch.core.Node("ur5_vel_start", "run_in_network_namespace_with_latency.py", 
        #    args='"sudo -u user /home/user/ursim-3.5.3.10825/start-ursim.sh"', 
        #    output="screen")
        #log("Created ursim node")
        
        #ariac_launch.runner.launch_node(ursim_node)
        #log("Launched ursim node")
        
        #time.sleep(30)
        
        #~ uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #~ roslaunch.configure_logging(uuid)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        rospy.wait_for_service('/gazebo/unpause_physics',5)
        self.unpause()
        rospy.sleep(3)
        self.pause()
        log("STARTED run!")


    def step(self, action):
        log("STEP BEGIN")
        done = False
        extra_data = {"digital_twin" : self.digital_twin}
        reward = ratio = midReward = scoreReward = 0
        
        rospy.wait_for_service('/gazebo/unpause_physics',5)
        self.latency_pub.publish('High' if action == 0 else 'Low')
        if self.digital_twin and self.tunnel_if:
            lat = 1000 if action == 0 else 20000 #15000
            log("Changing delay on eth1 to "+str(lat/1000))
            self.ipr.tc("change", "netem", self.tunnel_if, delay=lat)
            
        stepstart = rospy.get_time()
        self.unpause()
        rospy.loginfo("Running step")


        #!!! ha nem figment action szerint, akkor masodpercenkent !!!    
        rospy.sleep(1)
        #!!! figment action-hoz kell !!!
        #self.next_step_event.wait()

        if self.shutdown:
            extra_data = {"exit":True}
        if self.current_comp_state == 'done':
            log("GOT DONE MSG")
            done = True
            time.sleep(2)
            self.pause()
        #if not done:
            #self.pause()
        
        stepend = rospy.get_time()
        steptime = stepend-stepstart

        log("Start: {}, end: {}, duration: {}".format(stepstart, stepend, steptime))
        if action == 0:
            self.hAction += 1.0
            log("hAction: {}".format(self.hAction))
        else:
            self.lAction += 1.0
            log("lAction: {}".format(self.lAction))

        if self.prev_game_score != self.current_game_score and not done:
            
            # INTERMEDIATE SCORE REWARD
            scoreReward = ((self.current_game_score - self.prev_game_score)/self.max_score) * 2000
            log("intermediate score reward: "+str(scoreReward))
        self.prev_game_score = self.current_game_score
            
        # REWARD FOR ACTION
        midReward = -5 if action == 0 else 10
        if done:
            log("DONE")
            ratio = self.lAction/(self.lAction+self.hAction)*100.0
            log("LowQoC Ratio: {}".format(ratio))
            res, ttime, ptime =get_result()
            write_result(ratio,res, ttime, ptime)
            extra_data.update({"lowqoc_ratio": ratio, "game_score_diff":res-self.max_score})
            
            # REWARD AT END
            if res == self.max_score:
                reward = 2000 #- self.qReward
            else:
                reward = ((res - self.max_score)/self.max_score) * 2000 - 1000
            #self.qReward = 0
            self.lAction = self.hAction = 0.0
        else:
            reward = midReward + scoreReward
        log("Reward: {}".format(reward))
        log("Running scenario: {}".format(self.running_scenario))
        
        obs = self.get_obs()

        #!!! figment action-hoz kell !!!
        #self.next_step_event.clear()
        #log(str((obs, reward, done, extra_data)))
        return obs, reward, done, extra_data

    def reset(self):
        if self.started: 
            self.stop_all()
         
            self.started = False
            self.shutdown = False
        self.start_all()
        
        return self.get_obs()


    def render(self, mode="human", close=False):
        if close:
            tmp = os.popen("ps -Af").read()
            proccount = tmp.count('gzclient')
            if proccount > 0:
                if self.gzclient_pid != 0:
                    os.kill(self.gzclient_pid, signal.SIGTERM)
                    os.wait()
            return

        tmp = os.popen("ps -Af").read()
        proccount = tmp.count('gzclient')
        if proccount < 1:
            subprocess.Popen("gzclient")
            self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
        else:
            self.gzclient_pid = 0


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
