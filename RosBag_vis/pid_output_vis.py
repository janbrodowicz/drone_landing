import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

b = bagreader("pid_kryt_output.bag")
PID_X = b.message_by_topic("pid_x")
PID_Y = b.message_by_topic("pid_y")

df_pid_x = pd.read_csv(PID_X)
df_pid_y = pd.read_csv(PID_Y)
# print(df_pid_x[df_pid_x['Time']>3481])

# Below figure is done for PID output with critical gain
# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"][16:].tolist(), df_pid_x["data"][16:].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"][16:].tolist(), df_pid_y["data"][16:].tolist())
# ax[1].grid()
# plt.show()

# Below figure is done for PID type P after Ziegler-Nichols method
# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"].tolist(), df_pid_x["data"].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"].tolist(), df_pid_y["data"].tolist())
# ax[1].set_ylim([-0.4, 0.4])
# ax[1].grid()
# plt.show()

# Below figure is done for PID type PI after Ziegler-Nichols method
# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"][34:].tolist(), df_pid_x["data"][34:].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"][34:].tolist(), df_pid_y["data"][34:].tolist())
# ax[1].set_ylim([-0.2, 0.2])
# ax[1].grid()
# plt.show()

# Below figure is done for PID type PID after Ziegler-Nichols method
# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"].tolist(), df_pid_x["data"].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"].tolist(), df_pid_y["data"].tolist())
# ax[1].set_ylim([-0.4, 0.4])
# ax[1].grid()
# plt.show()
