import numpy as np
from common import get_args, experiment_setup
import tensorflow as tf
import time
import pickle5 as pickle
from envs import register_custom_envs


def train():
    register_custom_envs()
    args = get_args()
    env, env_test, agent, buffer, learner, tester = experiment_setup(args)

    args.logger.summary_init(agent.graph, agent.sess)

    # Progress info
    args.logger.add_item('Epoch')
    args.logger.add_item('Cycle')
    args.logger.add_item('Episodes@green')
    args.logger.add_item('Timesteps')
    args.logger.add_item('TimeCost(sec)')

    best_success = -1

    # Algorithm info
    for key in agent.train_info.keys():
        args.logger.add_item(key, 'scalar')

    # Test info
    for key in tester.info:
        args.logger.add_item(key, 'scalar')

    args.logger.summary_setup()
    goal_list = None

    for epoch in range(args.epochs):
        for cycle in range(args.cycles):
            args.logger.tabular_clear()
            args.logger.summary_clear()
            start_time = time.time()

            if args.learn == 'ghgg':
                goal_list = learner.learn(args, env, env_test, agent, buffer, write_goals=args.show_goals)
            else:
                learner.learn(args, env, env_test, agent, buffer)

            tester.cycle_summary()

            args.logger.add_record('Epoch', str(epoch) + '/' + str(args.epochs))
            args.logger.add_record('Cycle', str(cycle) + '/' + str(args.cycles))
            args.logger.add_record('Episodes', buffer.counter)
            args.logger.add_record('Timesteps', buffer.steps_counter)
            args.logger.add_record('TimeCost(sec)', time.time() - start_time)

            # Save learning progress to progress.csv file
            args.logger.save_csv()

            args.logger.tabular_show(args.tag)
            args.logger.summary_show(buffer.counter)

            # Save policy if new best_success was reached
            if args.logger.values["Success"] > best_success:
                best_success = args.logger.values["Success"]
                policy_file = args.logger.my_log_dir + "saved_policy-best"
                agent.save(policy_file)
                args.logger.info("Saved as best policy to {}!".format(policy_file))

        tester.epoch_summary()
        # Save periodic policy every epoch
        policy_file = args.logger.my_log_dir + "saved_policy"
        agent.saver.save(agent.sess, policy_file, global_step=epoch)
        args.logger.info("Saved periodic policy to {}!".format(args.logger.my_log_dir))

        # Plot current goal distribution for visualization (G-HGG only)
        if args.learn == 'ghgg' and goal_list and args.show_goals != 0:
            name = "{}goals_{}".format(args.logger.my_log_dir, epoch)
            with open('{}.pkl'.format(name), 'wb') as file:
                pickle.dump(goal_list, file)
            print('Goals saved')

    tester.final_summary()


if __name__ == '__main__':
    train()
