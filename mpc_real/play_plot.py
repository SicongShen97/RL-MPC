import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
# fig = plt.figure()
# gs = gridspec.GridSpec(3, 3, figure=fig)
#
# # 使用plt.subplot来作图, gs[0, :]表示这个图占第0行和所有列, gs[1, :2]表示这个图占第1行和第2列前的所有列,
# # gs[1:, 2]表示这个图占第1行后的所有行和第2列, gs[-1, 0]表示这个图占倒数第1行和第0列,
# # gs[-1, -2]表示这个图占倒数第1行和倒数第2列.
# # -----------------------
#
# ax6 = plt.subplot(gs[0, :])  # 第一个图，占尽了第一行和所有列
# ax6.plot([0,1],[1,2])
#
# ax7 = plt.subplot(gs[1, :2]) # 第二个图，占了第二列 和 第一列和第二列
#
#
# ax8 = plt.subplot(gs[1:, 2]) #
# # ax8.plot([0,1],[0,6])
# ax8.plot(np.arange(1, 13).reshape(6,2), color='red')
# plt.title("s")
#
# ax9 = plt.subplot(gs[-1, 0]) #
# ax9.scatter([0,1],[0,3])
#
# ax10 = plt.subplot(gs[-1, -2])
#
# plt.show()

fig = plt.figure()
plt.clf()
for i in range(10):
    plt.plot([0, 5], [i, i], 'b-')
    plt.pause(3)
    ax = fig.axes
    ax[0].get_lines().pop(-1).remove()
    ax[0].plot([0, 5], [i+1, i+1], 'b-')
    plt.draw()
    plt.pause(3)
plt.show()