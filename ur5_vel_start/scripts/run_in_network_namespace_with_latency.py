#!/usr/bin/env python

from __future__ import print_function

import sys
import signal
import os
import argparse
import subprocess
import time

setup = [
    # create a network namespace named "remote_location"
    'ip netns add remote_location',

    # create a pair of veth interfaces named 
    # "veth_local" and "veth_remote"
    'ip link add veth_local type veth peer name veth_remote',

    # Assign the "veth_remote" interface to the network namespace
    'ip link set netns remote_location veth_remote',

    # Assign an address to the "veth_local" interface
    'ip addr add 10.0.1.1/24 dev veth_local',
    'ip link set veth_local up',

    # Assign an address to the "inside" interface
    'ip netns exec remote_location ip addr add 10.0.1.2/24 dev veth_remote',
    'ip netns exec remote_location ip link set veth_remote up',
    'ip netns exec remote_location ip link set lo up',

]

setup_latency = [ 
    'tc qdisc add dev veth_local root netem  delay {delay}ms',
]

run_script = [ 
    'ip netns exec remote_location {script}'
]
    
change_latency = [ 
    'tc qdisc change dev veth_local root netem  delay {delay}ms',
]
    
teardown_latency = [ 
    'tc qdisc del dev veth_local root',
]

teardown = [
    'ip link del veth_local',
    'ip netns del remote_location',
]


def run_steps(steps, parameters, ignore_errors=False):
    for step in steps:
        sudo = "sudo -S".split()
        command = step.format(**parameters)
        print('RUNNING: {}'.format(command))
        command = command.split()
        try:
            p=subprocess.Popen(sudo+command, stderr=subprocess.STDOUT, stdin=subprocess.PIPE, shell=False)
            p.communicate(input=parameters['sudo_pw']+"\n")
        except KeyboardInterrupt:
            pass
            #p.send_signal(signal.SIGINT)
        if p.returncode != 0 and not ignore_errors:
            print("ERROR when running {}!\nExiting...".format(sudo+command))
            raise subprocess.CalledProcessError(1 if p.returncode is None else p.returncode, " ".join(sudo+command))
                
def prepare_arguments(parser):
    add = parser.add_argument
    add('script', help='Script to run')
    add('--delay', default=0, type=int, help='Set latency ')
        
def main(sysargv=None):
    sudo_pw = os.environ.get('SUDO_PASSWORD')
    if sudo_pw is None:
        print("Export your sudo password to SUDO_PASSWORD envvar")
        sys.exit(1)
        
    parser = argparse.ArgumentParser(
        description='Runs a script in a network namespace with latency.')
    prepare_arguments(parser)
    args = parser.parse_args(sysargv)
    parameters = vars(args)
    parameters.update({'sudo_pw':sudo_pw})
    
    try:
        run_steps(setup,parameters)
    
        if parameters["delay"]>0:
            run_steps(setup_latency,parameters)
            
        run_steps(run_script,parameters)
    except subprocess.CalledProcessError as ex:
        print(str(ex))
    finally:
        if parameters["delay"]>0:
            run_steps(teardown_latency,parameters,ignore_errors=True)

        run_steps(teardown,parameters, ignore_errors=True)
    
if __name__ == '__main__':
    # Filter out any special ROS remapping arguments.
    # This is necessary if the script is being run from a ROS launch file.
    import rospy
    filtered_argv = rospy.myargv(sys.argv)

    sys.exit(main(filtered_argv[1:]))
