import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns;

sns.set()
import os
import time

# Adopted source: Graph-Based Hindsight Goal Generation for Robotic Object Manipulation
# with Sparse-Reward Deep Reinforcement Learning, Matthias Brucker
from utils2.os_utils import make_dir


def plot_bars(config_names, env_id, config_labels):
    # set width of bar
    barWidth = 0.15
    fig, ax = plt.subplots()
    tols = [0,2,4]
    e_per_c = 200
    runs = [0,1,2]

    tol_config_names = []
    for config in config_names:
        for tol in tols:
            tol_config_names.append(config + '_tol' + str(tol))

    data = collect_data(tol_config_names, env_id, smooth=False)
    data_bars = {}

    for i in range(len(config_names)):
        config = config_names[i]
        print("Config: {}".format(config))
        data_bars[config] = []
        # merge curves from runs of one config
        for tol in tols:
            tol_config = config + '_tol' + str(tol)
            success_rates = []
            for run in runs:
                results = data[tol_config][run]
                x, y = results[0]
                success_rate = y[-1]
                success_rates.append(success_rate)

            mean = np.mean(success_rates)
            diffs_sq = [np.power(a - mean, 2) for a in success_rates]
            var = np.mean(diffs_sq, axis=0)
            std = np.sqrt(var)
            data_bars[config].append((mean, std))

    plt.xticks(fontsize=18)
    plt.yticks(fontsize=16)

    plt.grid(True)
    ax.set_axisbelow(True)

    # Make the plot
    for i in range(len(config_names)):
        config = config_names[i]
        label = config_labels[i]
        res = data_bars[config]
        means = []
        stds = []
        for bar in res:
            mean, std = bar
            means.append(mean)
            stds.append(std)

        br = np.arange(len(means)) + i * barWidth
        plt.bar(br, means, width=barWidth,
                edgecolor='white', label=label, linewidth=2, yerr=stds)

    # Adding Xticks
    plt.xlabel('Rates with tolerance of N collision', fontsize=18)
    plt.ylabel('Success rate', fontsize=18)
    plt.xticks([r + barWidth for r in range(3)],
               ['0', '2', '4'])

    plt.legend(loc=4, prop={'size': 14})
    plt.ylim((0, 1.0))
    plt.tight_layout()

    timestr = time.strftime("%m%d-%H%M")
    plt.savefig('log/figures/{}-col_{}.pdf'.format(env_id, timestr), format='pdf')

def load_results(file):
    if not os.path.exists(file):
        return None
    with open(file, 'r') as f:
        lines = [line for line in f]
    if len(lines) < 2:
        return None
    keys = [name.strip() for name in lines[0].split(',')]
    data = np.genfromtxt(file, delimiter=',', skip_header=1, filling_values=0.)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    assert data.ndim == 2
    assert data.shape[-1] == len(keys)
    result = {}
    for idx, key in enumerate(keys):
        result[key] = data[:, idx]
    return result


def smooth_reward_curve(x, y):
    halfwidth = int(np.ceil(len(x) / 60))  # Halfwidth of our smoothing convolution
    k = halfwidth
    xsmoo = x
    ysmoo = np.convolve(y, np.ones(2 * k + 1), mode='same') / np.convolve(np.ones_like(y), np.ones(2 * k + 1),
                                                                          mode='same')
    return xsmoo, ysmoo


def pad(xs, value=np.nan):
    maxlen = np.max([len(x) for x in xs])

    padded_xs = []
    for x in xs:
        if x.shape[0] >= maxlen:
            padded_xs.append(x)

        padding = np.ones((maxlen - x.shape[0],) + x.shape[1:]) * value
        x_padded = np.concatenate([x, padding], axis=0)
        assert x_padded.shape[1:] == x.shape[1:]
        assert x_padded.shape[0] == maxlen
        padded_xs.append(x_padded)
    return np.array(padded_xs)


def collect_data(config_names, env_id, smooth=True) -> dict:
    e_per_c = 200
    runs_count = 3

    data = {}
    for config in config_names:
        print('Load: {}'.format(config))
        for run_n in range(runs_count):
            print("\tRun: {}".format(run_n))
            curr_path = 'log/ddpg-{}-hgg/test_policy_{}_run{}.csv'.format(env_id, config, run_n)
            results = load_results(curr_path)

            success_rate = results['Success/mpc']
            samples = range(e_per_c)
            total_success_rate = []
            acc_success = 0.0

            for sample in samples:
                acc_success += success_rate[sample]
                total_success_rate.append(acc_success)

            x = np.array(samples) + 1
            y = np.array(total_success_rate)

            if smooth:
                x, y = smooth_reward_curve(x, y)

            run = run_n

            if config not in data:
                data[config] = {}
            if run not in data[config]:
                data[config][run] = []
            y_norm = y / e_per_c
            data[config][run].append((x, y_norm))

    return data


def plot_success_rate(config_names, env_id, config_labels):
    # collect data
    data = collect_data(config_names, env_id)

    fig, ax = plt.subplots()
    plt.clf()

    for i in range(len(config_names)):
        config = config_names[i]
        print("Config: {}".format(config))
        # merge curves from runs of one config
        results = sum([data[config][x] for x in data[config].keys()], [])

        xs, ys = zip(*results)
        xs, ys = pad(xs), pad(ys)
        assert xs.shape == ys.shape
        plt.plot(xs[0], np.nanmedian(ys, axis=0), label=config_labels[i])
        plt.fill_between(xs[0], np.nanpercentile(ys, 25, axis=0), np.nanpercentile(ys, 75, axis=0), alpha=0.25)

    plt.grid(True)
    ax.set_axisbelow(True)
    ax.yaxis.grid(color='white')

    plt.xticks(fontsize=14)
    plt.yticks(fontsize=16)

    #plt.title(env_id)
    plt.xlabel('Rollout', fontsize=18)
    plt.ylabel('Success rate (no collisions)', fontsize=16)
    plt.legend(loc=4)
    # plt.ylim((0, 1.0))
    plt.tight_layout()

    timestr = time.strftime("%m%d-%H%M")
    plt.savefig(os.path.join('log/figures/fig_test_{}_{}.pdf'.format(env_id, timestr)), format='pdf')


def fig1():
    plot_success_rate(['RLPolicy_tol0', 'MPCPolicy_tol0', 'MPCRLPolicy_tol0'],
                      'FetchPickDynSqrObstacle-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig2():
    plot_bars(['RLPolicy', 'MPCPolicy', 'MPCRLPolicy'],
                      'FetchPickDynSqrObstacle-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig3():
    plot_success_rate(['RLPolicy_tol0', 'MPCPolicy_tol0', 'MPCRLPolicy_tol0'],
                      'FetchPickDynObstaclesEnv-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig4():
    plot_bars(['RLPolicy', 'MPCPolicy', 'MPCRLPolicy'],
                      'FetchPickDynObstaclesEnv-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig5():
    plot_success_rate(['RLPolicy_tol0', 'MPCPolicy_tol0', 'MPCRLPolicy_tol0'],
                      'FetchPickDynObstaclesEnv-v2',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig6():
    plot_bars(['RLPolicy', 'MPCPolicy', 'MPCRLPolicy'],
                      'FetchPickDynObstaclesEnv-v2',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig7():
    plot_success_rate(['RLPolicy_tol0', 'MPCPolicy_tol0', 'MPCRLPolicy_tol0'],
                      'FetchPickDynLiftedObstaclesEnv-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])

def fig8():
    plot_bars(['RLPolicy', 'MPCPolicy', 'MPCRLPolicy'],
                      'FetchPickDynLiftedObstaclesEnv-v1',
                      ['exHGG', 'MPC', 'exHGG-MPC'])


if __name__ == '__main__':
    # plt.rcParams.update({'axes.facecolor': 'lavender'})
    make_dir('log', clear=False)
    make_dir('log/figures', clear=False)
    fig1()
    fig2()
    fig3()
    fig4()
    fig5()
    fig6()
    fig7()
    fig8()
    # fig9()
    # fig10()
    #fig11()
    #fig12()
