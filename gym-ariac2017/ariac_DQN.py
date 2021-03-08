#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import ray
from ray import tune
from ray.tune.tune import  run_experiments


def on_episode_end(info):
    episode = info["episode"]
    information = episode.last_info_for()
    episode.custom_metrics["lowqoc_ratio"] = information["lowqoc_ratio"]
    episode.custom_metrics["game_score_diff"] = information["game_score_diff"]

if __name__ == "__main__":

    experiment = {
        "ariac-dqn": {  # i.e. log to ~/ray_results/default
            "run": "DQN",
            #"checkpoint_freq": args.checkpoint_freq,
            #"keep_checkpoints_num": args.keep_checkpoints_num,
            #"checkpoint_score_attr": args.checkpoint_score_attr,
            #"local_dir": args.local_dir,
            #"resources_per_trial": (
            #    args.resources_per_trial and
            #    resources_to_json(args.resources_per_trial)),
            "stop": {
                "timesteps_total": 6000
            },
            "config": {
                "env": "gym_ariac2017:Ariac2017latency-v0",
                "batch_mode": "complete_episodes",
                "exploration_fraction": 0.3,
                "learning_starts": 1000,
                "num_workers": 1,
                "target_network_update_freq": 120,
                "timesteps_per_iteration": 60,
                "train_batch_size": 200,
                "num_cpus_for_driver": 0,
                "lr": tune.grid_search([1e-5, 1e-4, 5e-4, 1e-3, 1e-2, 1e-1]),
                "adam_epsilon": tune.grid_search([0.001, 0.01, 0.05, 0.1, 1]),
                "callbacks": {
                    "on_episode_end": tune.function(on_episode_end),
                },
            },
            #"restore": args.restore,
            "num_samples": 1,
            #"upload_dir": args.upload_dir,
        }
    }
    ray.init(num_cpus=1)
    tune.run_experiments(
        experiment
    )

