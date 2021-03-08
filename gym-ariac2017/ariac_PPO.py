#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import ray
import gym
import logging

import numpy as np

from ray import tune
from ray.tune.tune import  run_experiments

from ray.rllib.agents.ppo.ppo import PPOTrainer
from ray.rllib.agents.ppo.ppo_policy import PPOTFPolicy, ppo_surrogate_loss
from ray.rllib.evaluation.worker_set import WorkerSet
from ray.rllib.evaluation.rollout_worker import RolloutWorker

from ray.rllib.utils import try_import_tf

tf = try_import_tf()


from ray.rllib.evaluation.postprocessing import compute_advantages, \
    Postprocessing
from ray.rllib.policy.sample_batch import SampleBatch
from ray.rllib.models.catalog import ModelCatalog

logger = logging.getLogger(__name__)

ROBOT_IP = "192.168.10.1"

DIGITAL_TWIN = "digital_twin"

class WorkerSet_dt(WorkerSet):
    def add_workers(self, num_workers):
        """Create and add a number of remote workers to this worker set."""
        remote_args = {
            "num_cpus": self._remote_config["num_cpus_per_worker"],
            "num_gpus": self._remote_config["num_gpus_per_worker"],
            "resources": self._remote_config["custom_resources_per_worker"],
        }
        cls = RolloutWorker.as_remote(**remote_args).remote
        self._remote_workers.extend([
            self._make_worker(cls, self._env_creator, self._policy, i + 1,
                              self._remote_config) for i in range(num_workers)
        ])

def make_workers_with_dt(trainer, env_creator, policy, config):
    env_chooser = lambda env_config : gym.make(config["env"], digital_twin=True, robot_ip=ROBOT_IP) if env_config.worker_index == 1 else env_creator(env_config)
    return WorkerSet_dt(
            env_chooser,
            policy,
            config,
            num_workers=config["num_workers"],
            logdir=trainer.logdir)
            
# Frozen logits of the policy that computed the action
BEHAVIOUR_LOGITS = "behaviour_logits"


class PPOLoss(object):
    def __init__(self,
                 action_space,
                 value_targets,
                 advantages,
                 actions,
                 logits,
                 vf_preds,
                 curr_action_dist,
                 value_fn,
                 cur_kl_coeff,
                 valid_mask,
                 entropy_coeff=0,
                 clip_param=0.1,
                 vf_clip_param=0.1,
                 vf_loss_coeff=1.0,
                 use_gae=True):
        """Constructs the loss for Proximal Policy Objective.

        Arguments:
            action_space: Environment observation space specification.
            value_targets (Placeholder): Placeholder for target values; used
                for GAE.
            actions (Placeholder): Placeholder for actions taken
                from previous model evaluation.
            advantages (Placeholder): Placeholder for calculated advantages
                from previous model evaluation.
            logits (Placeholder): Placeholder for logits output from
                previous model evaluation.
            vf_preds (Placeholder): Placeholder for value function output
                from previous model evaluation.
            curr_action_dist (ActionDistribution): ActionDistribution
                of the current model.
            value_fn (Tensor): Current value function output Tensor.
            cur_kl_coeff (Variable): Variable holding the current PPO KL
                coefficient.
            valid_mask (Tensor): A bool mask of valid input elements (#2992).
            entropy_coeff (float): Coefficient of the entropy regularizer.
            clip_param (float): Clip parameter
            vf_clip_param (float): Clip parameter for the value function
            vf_loss_coeff (float): Coefficient of the value function loss
            use_gae (bool): If true, use the Generalized Advantage Estimator.
        """

        def reduce_mean_valid(t):
            return tf.reduce_mean(tf.boolean_mask(t, valid_mask))

        dist_cls, _ = ModelCatalog.get_action_dist(action_space, {})
        prev_dist = dist_cls(logits)
        # Make loss functions.
        logp_ratio = tf.exp(
            curr_action_dist.logp(actions) - prev_dist.logp(actions))
        action_kl = prev_dist.kl(curr_action_dist)
        self.mean_kl = reduce_mean_valid(action_kl)

        curr_entropy = curr_action_dist.entropy()
        self.mean_entropy = reduce_mean_valid(curr_entropy)

        surrogate_loss = tf.minimum(
            advantages * logp_ratio,
            advantages * tf.clip_by_value(logp_ratio, 1 - clip_param,
                                          1 + clip_param))
        self.mean_policy_loss = reduce_mean_valid(-surrogate_loss)

        if use_gae:
            vf_loss1 = tf.square(value_fn - value_targets)
            vf_clipped = vf_preds + tf.clip_by_value(
                value_fn - vf_preds, -vf_clip_param, vf_clip_param)
            vf_loss2 = tf.square(vf_clipped - value_targets)
            vf_loss = tf.maximum(vf_loss1, vf_loss2)
            self.mean_vf_loss = reduce_mean_valid(vf_loss)
            loss = reduce_mean_valid(
                -surrogate_loss + cur_kl_coeff * action_kl +
                vf_loss_coeff * vf_loss - entropy_coeff * curr_entropy)
        else:
            self.mean_vf_loss = tf.constant(0.0)
            loss = reduce_mean_valid(-surrogate_loss +
                                     cur_kl_coeff * action_kl -
                                     entropy_coeff * curr_entropy)
        self.loss = loss


def ppo_surrogate_loss_dt(policy, batch_tensors):
    print("LOSS INFOS: " + str(batch_tensors.keys()))
    if policy.model.state_in:
        max_seq_len = tf.reduce_max(
            policy.convert_to_eager(policy.model.seq_lens))
        mask = tf.sequence_mask(
            policy.convert_to_eager(policy.model.seq_lens), max_seq_len)
        mask = tf.reshape(mask, [-1])
    else:
        mask = tf.ones_like(
            batch_tensors[Postprocessing.ADVANTAGES], dtype=tf.bool)
            
    digital_twin = batch_tensors[DIGITAL_TWIN][0]

    policy.loss_obj = PPOLoss(
        policy.action_space,
        batch_tensors[Postprocessing.VALUE_TARGETS],
        batch_tensors[Postprocessing.ADVANTAGES],
        batch_tensors[SampleBatch.ACTIONS],
        batch_tensors[BEHAVIOUR_LOGITS],
        batch_tensors[SampleBatch.VF_PREDS],
        policy.action_dist,
        policy.convert_to_eager(policy.value_function),
        policy.convert_to_eager(policy.kl_coeff),
        mask,
        entropy_coeff=policy.config["entropy_coeff"],
        clip_param=policy.config["clip_param"]/2.0,
        vf_clip_param=policy.config["vf_clip_param"]/2.0,
        vf_loss_coeff=policy.config["vf_loss_coeff"],
        use_gae=policy.config["use_gae"])
        
    policy.loss_obj_dt = PPOLoss(
        policy.action_space,
        batch_tensors[Postprocessing.VALUE_TARGETS],
        batch_tensors[Postprocessing.ADVANTAGES],
        batch_tensors[SampleBatch.ACTIONS],
        batch_tensors[BEHAVIOUR_LOGITS],
        batch_tensors[SampleBatch.VF_PREDS],
        policy.action_dist,
        policy.convert_to_eager(policy.value_function),
        policy.convert_to_eager(policy.kl_coeff),
        mask,
        entropy_coeff=policy.config["entropy_coeff"],
        clip_param=policy.config["clip_param"],
        vf_clip_param=policy.config["vf_clip_param"],
        vf_loss_coeff=policy.config["vf_loss_coeff"],
        use_gae=policy.config["use_gae"])
    
    return tf.cond(digital_twin, true_fn=lambda: policy.loss_obj.loss, false_fn=lambda: policy.loss_obj_dt.loss)



def postprocess_ppo_gae_dt(policy,
                        sample_batch,
                        other_agent_batches=None,
                        episode=None):
    """Adds the policy logits, VF preds, and advantages to the trajectory."""

    completed = sample_batch["dones"][-1]
    trajsize = len(sample_batch[SampleBatch.ACTIONS])
    
    if("infos" in sample_batch and sample_batch["infos"][-1].get(DIGITAL_TWIN) is True):
        print("{} INFOS".format(trajsize))
        
        sample_batch[DIGITAL_TWIN] = np.ones_like(sample_batch[SampleBatch.ACTIONS],dtype=bool)
        assert trajsize == sample_batch[DIGITAL_TWIN].size
        print("SET DIGITAL_TWIN to TRUE")
    else:
        sample_batch[DIGITAL_TWIN] = np.zeros_like(sample_batch[SampleBatch.ACTIONS], dtype=bool)
        print("SET DIGITAL_TWIN to FALSE")

    if completed:
        last_r = 0.0
    else:
        next_state = []
        for i in range(len(policy.model.state_in)):
            next_state.append([sample_batch["state_out_{}".format(i)][-1]])
        last_r = policy._value(sample_batch[SampleBatch.NEXT_OBS][-1],
                               sample_batch[SampleBatch.ACTIONS][-1],
                               sample_batch[SampleBatch.REWARDS][-1],
                               *next_state)
    batch = compute_advantages(
        sample_batch,
        last_r,
        policy.config["gamma"],
        policy.config["lambda"],
        use_gae=policy.config["use_gae"])
    return batch
    
    
PPOTFPolicy_dt = PPOTFPolicy.with_updates(name="PPOTFPolicy_dt", loss_fn=ppo_surrogate_loss_dt, postprocess_fn=postprocess_ppo_gae_dt)
#PPOTrainer_dt = PPOTrainer.with_updates(name="PPOTrainer_dt", default_policy=PPOTFPolicy_dt, make_workers=make_workers_with_dt)
PPOTrainer_dt = PPOTrainer.with_updates(name="PPOTrainer_dt", default_policy=PPOTFPolicy_dt)


def on_episode_end(info):
    episode = info["episode"]
    information = episode.last_info_for()
    episode.custom_metrics["lowqoc_ratio"] = information["lowqoc_ratio"]
    episode.custom_metrics["game_score_diff"] = information["game_score_diff"]

if __name__ == "__main__":

    experiment = {
        "ariac-ppo-nodt": {  # i.e. log to ~/ray_results/default
            "run": PPOTrainer_dt,
            #"checkpoint_freq": args.checkpoint_freq,
            #"keep_checkpoints_num": args.keep_checkpoints_num,
            #"checkpoint_score_attr": args.checkpoint_score_attr,
            #"local_dir": args.local_dir,
            #"resources_per_trial": (
            #    args.resources_per_trial and
            #    resources_to_json(args.resources_per_trial)),
            "stop": {
                "timesteps_total": 30000
            },
            "checkpoint_at_end": True, 
            "config": {
                "env": "gym_ariac2017:Ariac2017latency-v1",
                "batch_mode": "complete_episodes",
                "num_workers": 1,
                "num_cpus_per_worker": 1,
                "train_batch_size": 512,
                "sample_batch_size": 64,
                "sgd_minibatch_size": 256,
                #"vf_clip_param": 100.0,
                "num_cpus_for_driver": 0,
                "callbacks": {
                    "on_episode_end": tune.function(on_episode_end),
                },
            },
            #"restore": args.restore,
            #"num_samples": 1,
            #"upload_dir": args.upload_dir,
        }
    }
    ray.init(num_cpus=1)
    tune.run_experiments(
        experiment,
        queue_trials=True
    )

