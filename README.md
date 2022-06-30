# Robot Navigation in Dynamic Environments using Reinforcement Learning and MPC

This is the implementation of exHGG-MPC algorithm of thesis: Robot Navigation in Dynamic Environments using 
Reinforcement Learning and MPC.
It is based on the implementation of the HGG paper [Exploration via Hindsight Goal Generation](http://arxiv.
org/abs/1906.04279) accepted by NeurIPS 2019. 

Why not MPC directly, but with RL: First, it is expensive to plan with MPC over large horizons. 
Second, we won't be able to grab different objects with a gripper. 
Also, what if you need to move two or more objects than you need to write a program which will apply MPC to different 
objects one by one, but RL can resolve this without additional programming.
In our case MPC is just a short time motion planner which ensures safety of RL actions.
MPC is not made to perform complicated tasks, only to correct RL actions.

## Features
- We use original HGG algorithm which we converted to support Tensorflow2
- HGG is trained on lower frequency (40ms / 20 sim iterations)
- MPC uses HGG as reference position and operates on higher frequency (10ms / 5 sim iterations)
- Simulation frequency is controlled by **env_n_substeps**
- Any other training algorithm can be used. You can also train original HGG or G-HGG (-- learn ghgg)
- MPC proposes an action which is then converted to a new position change target for MuJoCo.
- We trained HGG on MacOS with M1 Pro (requirements-macos.txt) since training on Intel without GPU was slow
- We evaluated HGG+MPC on Intel laptop (requirements-intel.txt) since ForcesPro works only on Intel

## Requirements
- Ubuntu/MacOS
- MuJoCO (version 2.1.1)
- Tensorflow 1.15.0 (ddpg2) or Tensorflow Metal
- ForcesPro

## Usage

(1) Setup ForcesPro using [installation guide](https://forces.embotech.com/Documentation/installation/python.html).
(2) Generate MPC solver. You will see a new folder generated.
```bash
cd mpc/
python pick_dyn_sqr_obstacles.py --mpc_gen t
```
(3) Train reinforcement learning policy.
```bash
python train2.py --alg ddpg2 --epochs 20 --tag='HGG_mpc7_M1' --env=FetchPickDynSqrObstacle-v1 
--reward_min -10 --goal mpc
```
(4) (optional) Run the scenario with exHGG (RLPolicy)
```bash
python play.py --env FetchPickDynSqrObstacle-v1 \
--play_path log/HGG_mpc7_M1-ddpg2-FetchPickDynSqrObstacle-v1-hgg/ \
--play_epoch 19 --goal mpc --play_policy RLPolicy --play_mode mujoco
```
(5) (optional) Run the scenario with MPC (MPCPolicy)
```bash
python play.py --env FetchPickDynSqrObstacle-v1 \
--play_path log/HGG_mpc7_M1-ddpg2-FetchPickDynSqrObstacle-v1-hgg/ \
--play_epoch 19 --goal mpc --play_policy MPCPolicy --timesteps 240 --play_mode mujoco --env_n_substeps 5
```
(6) Run the scenario with exHGG-MPC (MPCRLPolicy)
```bash
python play.py --env FetchPickDynSqrObstacle-v1 \
--play_path log/HGG_mpc7_M1-ddpg2-FetchPickDynSqrObstacle-v1-hgg/ \
--play_epoch 19 --goal mpc --play_policy MPCRLPolicy --timesteps 240 --play_mode mujoco --env_n_substeps 5
```

## Other scripts

Run the scenario with exHGG-MPC in plot (debug) mode (MPCRLPolicy)
```bash
python play.py --env FetchPickDynSqrObstacle-v1 \
--play_path log/HGG_mpc7_M1-ddpg2-FetchPickDynSqrObstacle-v1-hgg/ \
--play_epoch 19 --goal mpc --play_policy MPCRLPolicy --timesteps 240 --play_mode plot --env_n_substeps 5
```

Test success rate with collision tolerance (MPCRLPolicy). Results will be also saved in the log/ directory.
```bash
python test_policy.py --env FetchPickDynSqrObstacle-v1 --episodes 200 \
--play_path log/HGG_mpc7_M1-ddpg2-FetchPickDynSqrObstacle-v1-hgg/ --play_epoch 19 --goal mpc \
--play_policy MPCRLPolicy --test_col_tolerance 0 --env_n_substeps 5 --timesteps 240 --test_run_id 0
```

Train reinforcement learning policy with G-HGG algorithm. (Environment should provide **adapt_dict** dictionary).
For more details see the thesis Goal-Based Hindsight Goal Generation for Robotic Object Manipulation with 
Sparse-Reward Deep Reinforcement Learning (Matthias Brucker, 2020).
```bash
python train2.py --alg ddpg2 --epochs 20 --tag='HGG_ghgg_custom_M1' --env=FetchPickDynLabyrinthEnv-v1 \
--reward_min -10 --goal ghgg_custom --learn ghgg
```

Test environment in gym
```bash
python test_gym.py --env FetchPickDynSqrObstacle-v1 --goal mpc
```