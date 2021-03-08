https://github.com/openai/gym/blob/master/docs/creating-environments.md

#Install:
pip install -e .

	
#How digital twin PPO works currently
	in ariac_PPO.py:
		in main: experiment:  "run": PPOTrainer_dt,
		PPOTrainer_dt is custom trainer extending PPOTrainer with custom
		default_policy=PPOTFPolicy_dt, make_workers=make_workers_with_dt
		make_workers_with_dt: 
			custom env_chooser which checks the worker index and modifies the env_creator
			?MAYBE USE env_config to set up this?
			env_chooser = lambda env_config : gym.make(config["env"], digital_twin=True, robot_ip=ROBOT_IP) if env_config.worker_index == 1 else env_creator(env_config)
		PPOTFPolicy_dt is custom policy extending PPOTFPolicy with custom
		loss_fn=ppo_surrogate_loss_dt, postprocess_fn=postprocess_ppo_gae_dt
		postprocess_ppo_gae_dt:
			modified postprocess_ppo_gae that gets the infos from the env and if digital twin is True
			adds it to the trajectory
		ppo_surrogate_loss_dt:
			if trajectory has digital twin constructs a different PPOLoss than otherwise
	in ariac2017_latency.py gym env:
		reads the parameter digital_twin at __init__(), and robot_ip
		if true starts ur_modern_driver, and configures digital_twin plugin in gazebo
		when the latency changes in step() it uses pyroute2 tc to set a latency to the eth1 interface 
		which was added using docker network create, and which carries the ursim connection.
		the docker container needs the --cap-add=NET_ADMIN capability because it modifies the interface using tc
		
	
#Running:
	docker start head -ai
	docker exec -it head bash
	docker start w1 -ai
	
	roscore &
	rosparam set /use_sim_time true
	
	~/ariac_PPO.py


	
	
