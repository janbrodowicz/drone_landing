import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_output_landing_P.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/PID_bags/pid_P_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_PI_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_PID_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_kryt_output.bag")
# PID_X = b.message_by_topic("pid_x")
# PID_Y = b.message_by_topic("pid_y")

# df_pid_x = pd.read_csv(PID_X)
# df_pid_y = pd.read_csv(PID_Y)
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

# Below figure is done for PID type P landing
# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"].tolist(), df_pid_x["data"].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"].tolist(), df_pid_y["data"].tolist())
# # ax[1].set_ylim([-0.4, 0.4])
# ax[1].grid()
# plt.show()

#===========================================================================

b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/kalman_output_0.01_0.1_fixedbag.bag")

x_estimate = b.message_by_topic("estimate_x")
y_estimate = b.message_by_topic("estimate_y")
x_measurement = b.message_by_topic("measurement_x")
y_measurement = b.message_by_topic("measurement_y")
x_actual = b.message_by_topic("actual_x")
y_actual = b.message_by_topic("actual_y")

df_x_estimate = pd.read_csv(x_estimate)
df_y_estimate = pd.read_csv(y_estimate)
df_x_measurement = pd.read_csv(x_measurement)
df_y_measurement = pd.read_csv(y_measurement)
df_x_actual = pd.read_csv(x_actual)
df_y_actual = pd.read_csv(y_actual)
df_x_actual["data"] = df_x_actual["data"] * (-1)

# print(df_x_estimate)

fig, ax = plt.subplots(2)
ax[0].plot(df_x_estimate["Time"].tolist(), df_x_estimate["data"].tolist(), 'o-', markersize=2.5)
ax[0].plot(df_x_measurement["Time"].tolist(), df_x_measurement["data"].tolist())
ax[0].plot(df_x_actual["Time"].tolist(), df_x_actual["data"].tolist())
ax[0].legend(["estimate", "measure", "actual"])
ax[0].grid()
ax[1].plot(df_y_estimate["Time"].tolist(), df_y_estimate["data"].tolist(), 'o-', markersize=2.5)
ax[1].plot(df_y_measurement["Time"].tolist(), df_y_measurement["data"].tolist())
ax[1].plot(df_y_actual["Time"].tolist(), df_y_actual["data"].tolist())
ax[1].legend(["estimate", "measure", "actual"])
ax[1].grid()
plt.show()


# plt.plot(df_x_actual["data"].tolist(), df_y_actual["data"].tolist())
# plt.grid()
# plt.show()