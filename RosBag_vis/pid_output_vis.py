import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd

# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_output_landing_P.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/PID_bags/pid_P_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_PI_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/pid_PID_output.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/PID_bags/pid_output.bag")
# PID_X = b.message_by_topic("pid_x")
# PID_Y = b.message_by_topic("pid_y")

# df_pid_x = pd.read_csv(PID_X)
# df_pid_y = pd.read_csv(PID_Y)
# # print(df_pid_x[df_pid_x['Time']>3481])

# fig, ax = plt.subplots(2)
# ax[0].plot(df_pid_x["Time"].tolist(), df_pid_x["data"].tolist())
# ax[0].grid()
# ax[1].plot(df_pid_y["Time"].tolist(), df_pid_y["data"].tolist())
# ax[1].grid()
# plt.show()

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

#========================================================================================================================
#========================================================================================================================

# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/kalman_output_0.01_0.1_fixedbag.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/Kalman_CV_with_vel_measure/kalman_output_changed_impl_(R=1, P=0.1, w=0.001, input=0.0001)_mes.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/Kalman_CV_without_vel_measure/kalman_output_(R=1, P=0.1, w=0.0001, input=0.0001).bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/Kalman_CA/kalman_output_(R=0.1).bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/Kalman_CP/kalman_output_changed_implementation.bag")
# b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Kalman_bags/Kalman_moving_pad/kalman_output.bag")

# x_estimate = b.message_by_topic("estimate_x")
# y_estimate = b.message_by_topic("estimate_y")
# x_measurement = b.message_by_topic("measurement_x")
# y_measurement = b.message_by_topic("measurement_y")
# x_actual = b.message_by_topic("actual_x")
# y_actual = b.message_by_topic("actual_y")

# x_estimate = b.message_by_topic("x_estimate")
# y_estimate = b.message_by_topic("y_estimate")
# x_measurement = b.message_by_topic("x_measurement")
# y_measurement = b.message_by_topic("y_measurement")
# x_actual = b.message_by_topic("x_actual")
# y_actual = b.message_by_topic("y_actual")

# df_x_estimate = pd.read_csv(x_estimate)
# df_y_estimate = pd.read_csv(y_estimate)
# df_x_measurement = pd.read_csv(x_measurement)
# df_y_measurement = pd.read_csv(y_measurement)
# df_x_actual = pd.read_csv(x_actual)
# df_y_actual = pd.read_csv(y_actual)
# df_x_actual["data"] = df_x_actual["data"] * (-1)

# print(df_x_estimate)

# fig, ax = plt.subplots(2)
# ax[0].plot(df_x_estimate["Time"].tolist(), df_x_estimate["data"].tolist(), 'o-', markersize=2.2)
# ax[0].plot(df_x_measurement["Time"].tolist(), df_x_measurement["data"].tolist(), 'ko-', markersize=2.2)
# # ax[0].plot(df_x_actual["Time"].tolist(), df_x_actual["data"].tolist())
# ax[0].legend(["estimate", "measure", "actual"])
# ax[0].grid()
# ax[1].plot(df_y_estimate["Time"].tolist(), df_y_estimate["data"].tolist(), 'o-', markersize=2.2)
# ax[1].plot(df_y_measurement["Time"].tolist(), df_y_measurement["data"].tolist(), 'ko-', markersize=2.2)
# # ax[1].plot(df_y_actual["Time"].tolist(), df_y_actual["data"].tolist())
# ax[1].legend(["estimate", "measure", "actual"])
# ax[1].grid()
# plt.show()


# plt.plot(df_x_actual["data"].tolist(), df_y_actual["data"].tolist())
# plt.grid()
# plt.show()

# mean_x = sum(df_x_measurement['data'].to_list()) / len(df_x_measurement['data'].to_list())
# war_x = 0
# for data in df_x_measurement['data'].to_list():
#     war_x += (data - mean_x) ** 2
# war_x = war_x / (len(df_x_measurement['data'].to_list()))
# print(war_x)


# mean_y = sum(df_y_measurement['data'].to_list()) / len(df_y_measurement['data'].to_list())
# war_y = 0
# for data in df_y_measurement['data'].to_list():
#     war_y += (data - mean_y) ** 2
# war_y = war_y / (len(df_y_measurement['data'].to_list()))
# print(war_y)

# err_x_measure = sum(np.subtract(df_x_measurement['data'].to_list(), df_x_actual['data'].to_list()) ** 2) / len(df_x_measurement['data'].to_list())
# err_x_estimate = sum(np.subtract(df_x_estimate['data'].to_list(), df_x_actual['data'].to_list()) ** 2) / len(df_x_estimate['data'].to_list())

# print("Error for x measurement: ", err_x_measure)
# print("Error for x estimate: ", err_x_estimate)


#========================================================================================================================
#========================================================================================================================


b = bagreader("/home/jan/catkin_ws/src/ros_landing/RosBag_vis/Position/drone_pad_pos_circle_1_(very_good).bag")

x_drone = b.message_by_topic("drone_x")
y_drone = b.message_by_topic("drone_y")
z_drone = b.message_by_topic("drone_z")

x_pad = b.message_by_topic("pad_x")
y_pad = b.message_by_topic("pad_y")
z_pad = b.message_by_topic("pad_z")


df_x_drone = pd.read_csv(x_drone)
df_y_drone = pd.read_csv(y_drone)
df_z_drone = pd.read_csv(z_drone)

df_x_pad = pd.read_csv(x_pad)
df_y_pad = pd.read_csv(y_pad)
df_z_pad = pd.read_csv(z_pad)


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.plot(df_x_drone['data'].to_list(), df_y_drone['data'].to_list(), df_z_drone['data'].to_list())
ax.plot(df_x_pad['data'].to_list(), df_y_pad['data'].to_list(), df_z_pad['data'].to_list())
# ax.set_ylim(-1, 1)
plt.show()

# plt.plot(df_x_drone['data'].to_list(), df_y_drone['data'].to_list())
# plt.plot(df_x_pad['data'].to_list(), df_y_pad['data'].to_list())
# # plt.ylim([-0.4, 0.4])
# plt.grid()
# plt.show()

# print(b.topic_table)