# Rosbags in PID_bags folder

* pid_kryt_output.bag contains oscilation around landing pad output for critical gain k_crit = 0.8 (X and Y axis) 

## After Ziegler-Nichols method

* pid_P_output.bag contains oscilation around landing pad output for P controller (Kp=0.4)
	- for P it works well on high altitudes and also on low but worse than PID
* pid_PI_output.bag contains oscilation around landing pad output for PI controller (Kp=0.36, Ki=0.094)
	- for PI it works well on low altitude but on high it becomes very unstable, but it's not smooth
* pid_PID_output.bag contains oscilation around landing pad output for PID controller (Kp=0.48, Ki=0.213, Kd=0.259)
	- for PID it works well on low altitude but on high it becomes very unstable 
