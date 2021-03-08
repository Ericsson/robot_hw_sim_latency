from gym.envs.registration import register

# ariac2017-latency
register(
    id='Ariac2017latency-v1',
    entry_point='gym_ariac2017.envs.ariac2017_latency:Ariac2017LatencyEnv',
    # CUSTOM ARGS
    kwargs={'digital_twin' : False, "robot_ip": "0.0.0.0"},
)


