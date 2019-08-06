#!/usr/bin/env python3

import sys
import os
import signal
import yaml
import io
import asyncio
import pathlib
import ssl
import websockets
import json
import rospkg
import rospy
import roslaunch
import subprocess
import time
from functools import partial
from std_srvs.srv import Empty
from std_msgs.msg import String
from asyncio import Event
#from threading import Event

rospack = rospkg.RosPack()

default_config = {
'orders': {
    'order_0': {
        'announcement_condition_value': 0.0, 
        'kit_count': 1, 
        'announcement_condition': 'time', 
        'parts': {
            'part_1': {
                'type': 'order_part2', 
                'pose': {'xyz': [-0.12, -0.2, 0], 'rpy': [0, 0, 0]}
                }, 
            'part_0': {
                'type': 'order_part1', 
                'pose': {'xyz': [0.1, -0.18, 0], 'rpy': [0, 0, 0]}
                }
        }
    }
}, 

'time_limit': 500, 
'options': {
    'gazebo_state_logging': False, 
    'model_type_aliases': {
        'order_part3': 'pulley_part', 
        'order_part1': 'piston_rod_part', 
        'order_part2': 'gear_part', 
        'order_part4': 'disk_part'
    }, 
    'insert_models_over_bins': True, 
    'fill_demo_tray': False,
    'belt_population_cycles': 8
}
}

scenarios = {
'scenario1':   {
    'models_over_bins': {
        'bin6': {
            'models': {
                'gear_part': {
                    'num_models_x': 4, 
                    'num_models_y': 4, 
                    'xyz_start': [0.13, 0.13, 0.0], 
                    'xyz_end': [0.53, 0.53, 0.0], 
                    'rpy': [0, 0, 0]
                }
            }
        }, 
        'bin7': {
            'models': {
                'disk_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.14, 0.14, 0.0], 
                    'xyz_end': [0.46, 0.46, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }, 
        'bin5': {
            'models': {
                'piston_rod_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.21, 0.21, 0.0], 
                    'xyz_end': [0.41, 0.41, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }
    }
},

'scenario2':   {
    'models_over_bins': {
        'bin6': {
            'models': {
                'gear_part': {
                    'num_models_x': 4, 
                    'num_models_y': 4, 
                    'xyz_start': [0.13, 0.13, 0.0], 
                    'xyz_end': [0.53, 0.53, 0.0], 
                    'rpy': [0, 0, 0]
                }
            }
        }, 
        'bin7': {
            'models': {
                'disk_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.14, 0.14, 0.0], 
                    'xyz_end': [0.46, 0.46, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }, 
        'bin5': {
            'models': {
                'piston_rod_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.21, 0.21, 0.0], 
                    'xyz_end': [0.41, 0.41, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }, 
        'bin8': {
            'models': {
                'pulley_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.14, 0.14, 0.0], 
                    'xyz_end': [0.46, 0.46, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }
    }
},
'scenario3': {
    "belt_parts":{
        "gear_part":{
            8:{
                "pose":{
                    "xyz": [0.0, 0.0, 0.1],
                    "rpy": [0, 0, 0]
                }
            }
        }
    },
    'models_over_bins': {
        'bin7': {
            'models': {
                'disk_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.14, 0.14, 0.0], 
                    'xyz_end': [0.46, 0.46, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }, 
        'bin5': {
            'models': {
                'piston_rod_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.21, 0.21, 0.0], 
                    'xyz_end': [0.41, 0.41, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }, 
        'bin8': {
            'models': {
                'pulley_part': {
                    'num_models_x': 2, 
                    'num_models_y': 2, 
                    'xyz_start': [0.14, 0.14, 0.0], 
                    'xyz_end': [0.46, 0.46, 0.0], 
                    'rpy': [0, 0, 'pi/4']
                }
            }
        }
    }
}
    
}

drop_zones = {
    'drop_regions': {
        'agv1_unreachable_1': {
            'destination': {
                'rpy': [0, 0, 0.2],
                'xyz': [0.6, 3.1, 1.0]
            },
            'max': {
                'xyz': [0.6, 3.5, 1.5]
            },
            'min': {
                'xyz': [0.0, 2.7, 0.6]
            },
            'part_type_to_drop': 'piston_rod_part'
        },
        'agv1_unreachable_2': {
            'destination': {
                'rpy': [0, 0, 0.15],
                'xyz': [0.6, 3.1, 1.0]
            },
            'max': {
                'xyz': [0.6, 3.5, 1.5]
            },
            'min': {
                'xyz': [0.0, 2.7, 0.6]
            },
            'part_type_to_drop': 'gear_part'
        },
        'agv2_unreachable_1': {
            'destination': {
                'rpy': [0, 0, -0.2],
                'xyz': [0.6, -3.1, 1.0]
            },
            'max': {
                'xyz': [0.6, -3.5, 1.5]
            },
            'min': {
                'xyz': [0.0, -2.7, 0.6]
            },
            'part_type_to_drop': 'piston_rod_part'
        },
        'agv2_unreachable_2': {
            'destination': {
                'rpy': [0, 0, -0.15],
                'xyz': [0.6, -3.1, 1.0]
            },
            'max': {
                'xyz': [0.6, -3.5, 1.5]
            },
            'min': {
                'xyz': [0.0, -2.7, 0.6]
            },
            'part_type_to_drop': 'gear_part'
        }
    }
}

tray_positions = [
    [0.1,0.2,0],
    [0.1,-0.2,0],
    [-0.1,-0.2,0],
    [-0.1,0.2,0],
]

# Keeping track of how many positions are taken in the agvs
order_num = [0,0];

def create_part(num_order, part_type):
    global order_num
    
    if order_num[num_order] > 3:
        return None, {}
    nextpos = tray_positions[order_num[num_order]]
    order_num[num_order]+=1
    part = {
        'type': part_type, 
        'pose': {'xyz': nextpos, 'rpy': [0, 0, 0]}}
    return "part_"+str(order_num[num_order]-1), part


def create_order(num_order=0, time=0.0, parts=[]):
    if not parts :
        return {}
    parts_list = list(map(partial(create_part,num_order),parts))
    #print(parts_list)
    order = {
            'announcement_condition_value': time, 
            'kit_count': 1, 
            'announcement_condition': 'time', 
            'parts': {part_name:part for part_name, part in parts_list if part_name is not None}
            }
    print("Creating order:", order)
    return order


json_name_to_part_name = {
    "pistons":"piston_rod_part",
    "gears":"gear_part",
    "discs":"disk_part",
    "pulleys":"pulley_part",
}

def prepare_parameters(params):
    high = []
    low = []
    options = {}
    for key, item in params.items():
        prio, _, remainder = key.partition("-")
        value = item['value']
        if prio == "low":
            for i in range(int(value)):
                low.append(json_name_to_part_name[remainder])
        elif prio == "high":
            if remainder in json_name_to_part_name.keys():
                for i in range(int(value)):
                    high.append(json_name_to_part_name[remainder])
            else:
                #handle high-time
                try:
                    value = float(value)
                except ValueError:
                    pass
                options[key] = value
        else:
            options[key] = value
    return low, high, options
    
def generate_config(message_params):
    config = dict(default_config)
    #print(message_params)
    low, high, options = prepare_parameters(message_params)
    print(low, high, options)
    low_order = create_order(0,0.0,low)
    high_order = create_order(1,options["high-time"],high)
    

    config['orders']['order_0'] = low_order
    if high_order:
        config['orders']['order_1'] = high_order
        
    if options['failures']:
        config['drops'] = drop_zones
    
    config.update(scenarios[options["scenarios"]])
        
    #print(config) 
    
    yaml.Dumper.ignore_aliases = lambda *args : True
    yaml.dump(config,io.open("/tmp/digitaltwindemo/setup.yaml",'w'))
    return options

def comp_state_callback(loop, msg):
    global stopping_event
    if msg.data == 'done': 
        print("Done message get")
        # If 'done' signal the main server loop (handler) that it can shutdown the competition environment
        loop.call_soon_threadsafe(stopping_event.set)
        #stopping_event.set()
        
def qoc_callback(latency, msg):
    global robot_ip
    if msg.data == "High":
        print("~~~~~~HIGH QOC~~~~~~")
        run_steps(change_latency,{"delay":"0"})
        subprocess.check_output(
            ["ping","-p","ff","-Q","01","-s","1","-c","1",robot_ip])
    else:
        print("~~~~~~LOW QOC~~~~~~")
        run_steps(change_latency,{"delay":latency})
        subprocess.check_output(
            ["ping","-p","00","-Q","01","-s","1","-c","1",robot_ip])
            
async def send_message(text, bold=False):
    global gl_websocket
    print(text)
    style = "normal" if not bold else "bold"
    await gl_websocket.send(json.dumps({"messagetype":"MESSAGE","style":style, "data":text}))
    
#async def wait_for_result():
    #global competition_result
    ##await send_message('Wait for result started')
    #with (pathlib.Path.home() / ".gazebo" / "server-11345" / "default.log").open() as logfile:
        #await send_message('Opened logfile')
        #logfile.seek(0,2)      # Go to the end of the file
        #while True:
             #line = logfile.readline()
             #if not line:
                 #asyncio.sleep(0.1)    # Sleep briefly
                 #continue
             #await send_message(line)
async def get_result():
    global competition_result
    score = ""
    with (pathlib.Path.home() / ".gazebo" / "server-11345" / "default.log").open() as logfile:
        inscore = False
        
        for line in logfile:
            if line.startswith("<game_score>"):
                inscore = True
            if inscore:
                score+=line
            if line.startswith("</game_score>"):
                break
    await send_message(score, bold=True)
    #return score
            
async def run_competition(options):
    global ariac_launch
    global ur_launch
    global comp_state_sub
    global qoc_sub
    global unpause
    global pause
    global figment_process
    
    global robot_ip
    
    run_steps(change_latency,{"delay":options["latency"]})
    
    await send_message("Setting latency: "+options["latency"]+"ms")
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    
    await send_message("Starting competition")
    
    # Start the competition environment
    ariac_roslaunch_file = [(osrf_gear_folder+"/launch/sample_environment.launch")]
    #ariac_roslaunch_file = [(cygment_folder+"/launch/qual_a_2.launch")]
    ariac_launch = roslaunch.parent.ROSLaunchParent(uuid, ariac_roslaunch_file , is_core=False, verbose=True)
    print("Created ariac ROSLaunch")
    ariac_launch.start(auto_terminate=True)
    print("Started ariac roslaunch")
    
    await send_message("Started ARIAC environment")
    
    
    #rospy.on_shutdown(self.on_shutdown)

    # Subscribe to topic that reports the state of the competition( eg: is it 'done')
    comp_state_sub = rospy.Subscriber("/ariac/competition_state", String, partial(comp_state_callback,asyncio.get_event_loop()))

    #self.figment_action_sub = rospy.Subscriber("/figment/action", String, self.action_callback)
    #self.latency_pub = rospy.Publisher('/latency_plugin_simple_queue/priority', String, queue_size=1)
    #self.comp_state_sub = rospy.Subscriber("/ariac/competition_state", String, self.comp_state_callback)
   
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    
    rospy.wait_for_service('/gazebo/unpause_physics', 15)
    time.sleep(10)

    figment_node = roslaunch.core.Node("figment_ariac", "scheduler_plan",
    #figment_node = roslaunch.core.Node("cygment", "cygment_node",
        output="screen")
    print("Created figment node")

    # Start the figment solver code
    figment_process, success = ariac_launch.runner.launch_node(figment_node)
    print("Launched figment node success:"+str(success))
    await send_message("Started competition solver")

    time.sleep(2)

    if options["qoc"]:
        qoc_sub = rospy.Subscriber("/figment/trajectory/precision", String, partial(qoc_callback,options["latency"]))
    else:
        qoc_sub = None

    time.sleep(3)
    #Step simulation 4 times to start the controllers
    subprocess.call(["gz","world","-m","1"])

    # Start ur_modern_driver
    ur_roslaunch_file = [(ur5_vel_start_folder+"/launch/setup.launch", ["robot_ip:="+robot_ip, 
                                                                        "sim:=false", 
                                                                        "ns:=realur/", 
                                                                        "moveit:=false",
                                                                        "latency_plugin:=true"]
                        )]        
    ur_launch = roslaunch.parent.ROSLaunchParent(uuid, ur_roslaunch_file , is_core=False, verbose=True)
    print("Created ur ROSLaunch")
    ur_launch.start(auto_terminate=True)
    print("Started ur roslaunch")
    await send_message("Started UR controller")


    subprocess.call(["gz","world","-m","1"])
    #asyncio.get_event_loop().create_task(wait_for_result())



    time.sleep(2)

    subprocess.call(["gz","world","-m","4"])

    time.sleep(2)

    await send_message("Unpausing simulation")
    unpause()

async def stop_competition():
    global ariac_launch
    global ur_launch
    global comp_state_sub
    global unpause
    global pause
    global figment_process
    global stopping_event
    global order_num

    await send_message("Stopping competition")

    if comp_state_sub is not None:
        comp_state_sub.unregister()
    if qoc_sub is not None:
        qoc_sub.unregister()
    if unpause is not None:
        unpause.close()
    if pause is not None:
        pause.close()
    if figment_process is not None:
        figment_process.stop()
        while figment_process.is_alive():
            print("Waiting for figment stop!")
            await asyncio.sleep(1)

    if ariac_launch is not None:
        ariac_launch.shutdown()
        while not ariac_launch.runner.pm.done:
            print("Waiting for ariac_launch to stop!")
            await asyncio.sleep(1)
        ariac_launch = None
    if ur_launch is not None:
        ur_launch.shutdown()
        while not ur_launch.runner.pm.done:
            print("Waiting for ur_launch to stop!")
            await asyncio.sleep(1)
        ur_launch = None

    order_num = [0,0];
    stopping_event.clear()
    await send_message("Stopped competition!")



gl_websocket = None
competition_result = None
# Main server loop
async def handler(websocket, path):
    global gl_websocket
    gl_websocket = websocket
    while True:
        message = None

        while True:
            m = await websocket.recv()
            message = json.loads(m)
            if message["messagetype"] == "START":
                break
                
        
        await send_message("Generating config!")

        options = generate_config(message["params"])
        await run_competition(options)
        await stopping_event.wait()
        print("Stopping event get!!")
        await asyncio.sleep(3)
        await get_result()
        print("Wait done!")
        await stop_competition()

        await send_message("Finishing demo...")
        await asyncio.sleep(10)

        print("Done, sending END")
        await websocket.send(json.dumps({"messagetype":"END"}))

osrf_gear_folder = None
figment_folder = None
ur5_vel_start_folder = None
stopping_event = Event()
comp_state_sub = None
qoc_sub = None
ariac_launch = None
ur_launch = None
figment_process = None
unpause = None
pause = None

robot_ip = "172.31.32.232"

tc_params = {
    "robot_if": None,
    "sudo_pw": None
}

setup_latency = [ 
    'tc qdisc add dev {robot_if} root netem  delay 0ms',
]

change_latency = [ 
    'tc qdisc change dev {robot_if} root netem  delay {delay}ms',
]

teardown_latency = [ 
    'tc qdisc del dev {robot_if} root',
]

def run_steps(steps, params, ignore_errors=False):
    global tc_params
    parameters = dict(tc_params)
    parameters.update(params)
    for step in steps:
        sudo = 'sudo -S'.split()
        command = step.format(**parameters)
        print('RUNNING: {}'.format(command))
        command = command.split()
        try:
            p=subprocess.Popen(sudo+command, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, shell=False, universal_newlines=True)
            p.communicate(input=parameters['sudo_pw']+"\n")
        except KeyboardInterrupt:
            pass
            #p.send_signal(signal.SIGINT)
        if p.returncode != 0 and not ignore_errors:
            print("ERROR when running {}!\nExiting...".format(sudo+command))
            raise subprocess.CalledProcessError(1 if p.returncode is None else p.returncode, " ".join(sudo+command))

async def ask_exit(signame, loop):
    print("got signal %s: exit" % signame)
    await stop_competition()
    rospy.signal_shutdown("Shutting down")
    run_steps(teardown_latency,{})

    loop.stop()

def get_if(ip):
    res = subprocess.check_output(["ip", "route", "get", ip])
    res = res.decode().split(' ')[2]
    return res

def main(sysargv=None):
    global robot_ip
    global sudo_pw
    global tc_params
    global osrf_gear_folder
    global figment_folder
    #global cygment_folder
    global ur5_vel_start_folder
    robot_ip = sysargv[0]
    tc_params["robot_if"] = get_if(robot_ip)

    print(tc_params["robot_if"])

    tc_params["sudo_pw"] = os.environ.get('SUDO_PASSWORD')
    if tc_params["sudo_pw"] is None:
        print("Export your sudo password to SUDO_PASSWORD envvar")
        sys.exit(1)

    run_steps(setup_latency, {})

    osrf_gear_folder = rospack.get_path('osrf_gear')
    figment_folder = rospack.get_path('figment_ariac')
    #cygment_folder = rospack.get_path('cygment')
    ur5_vel_start_folder = rospack.get_path('ur5_vel_start')

    pathlib.Path('/tmp/digitaltwindemo').mkdir(parents=True, exist_ok=True)

    # Starts a node that will listen to messages from ariac
    rospy.init_node('websocketserver', anonymous=False, disable_signals=True)


    serverip = ''
    serverport = 8765

    start_server = websockets.serve(
        handler, serverip, serverport)

    loop=asyncio.get_event_loop()

    loop.run_until_complete(start_server)
    for signame in {'SIGINT', 'SIGTERM'}:
        loop.add_signal_handler(
                getattr(signal, signame),
                lambda: asyncio.ensure_future(ask_exit(signame, loop)))

    print("Started websocket server on", serverip, "port", serverport, "...")
    loop.run_forever()
    return 0

#def main(sysargv=None):
    #sudo_pw = os.environ.get('SUDO_PASSWORD')
    #if sudo_pw is None:
        #print("Export your sudo password to SUDO_PASSWORD envvar")
        #sys.exit(1)

    #parser = argparse.ArgumentParser(
        #description='Runs a script in a network namespace with latency.')
    #prepare_arguments(parser)
    #args = parser.parse_args(sysargv)
    #parameters = vars(args)
    #parameters.update({'sudo_pw':sudo_pw})

    #try:
        #run_steps(setup,parameters)

        #if parameters["delay"]>0:
            #run_steps(setup_latency,parameters)

        #run_steps(run_script,parameters)
    #except subprocess.CalledProcessError as ex:
        #print(str(ex))
    #finally:
        #if parameters["delay"]>0:
            #run_steps(teardown_latency,parameters,ignore_errors=True)

        #run_steps(teardown,parameters, ignore_errors=True)


if __name__ == '__main__':
    # Filter out any special ROS remapping arguments.
    # This is necessary if the script is being run from a ROS launch file.
    filtered_argv = rospy.myargv(sys.argv)

    sys.exit(main(filtered_argv[1:]))
