python play_real.py --env FrankaPickDynSqrObstacles-v1 \
--play_path log_real/ddpg2-FrankaPickDynSqrObstacles-v1-hgg/ \
--goal mpc --play_policy RLPolicy

python play_real.py --env FrankaPickDynSqrObstacles-v1 \
--play_path log_real/ddpg2-FrankaPickDynSqrObstacles-v1-hgg/ \
--goal mpc --play_policy MPCPolicy --mpc_gen False

python play_real.py --env FrankaPickDynSqrObstacles-v1 \
--play_path log_real/ddpg2-FrankaPickDynSqrObstacles-v1-hgg/ \
--goal mpc --play_policy MPCRLPolicy --mpc_gen False

# train
python train_real.py --env FrankaPickDynObstacles-v1 --tag 'simple_net'