#!/usr/bin/env python


def init():
	global gripper_state, logical_cameras, tf_manager 
	global belt_pieces, last_belt_piece, piece_pos, first_it 
	global piece_clock, CheckTaskIsFinished, piece_final_position 
	global partPicked, current_joint_state, agv1_status, agv2_status
	global speed_modifier

	gripper_state = None

	#Belt vars
	piece_final_position = 0
	first_it = True
	piece_pos = 0
	piece_clock = 0
	
	logical_cameras = {}
	belt_pieces = {}
	last_belt_piece = {}

	tf_manager = None
	CheckTaskIsFinished = False
	partPicked = False
	current_joint_state = None
	faulty_sensor1 = []
	faulty_sensor2 = []

	agv1_status = 'unknown'
	agv2_status = 'unknown'

	speed_modifier = 1
	
