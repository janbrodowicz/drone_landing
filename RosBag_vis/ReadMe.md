# Opis rosbagow zawartych w tym folderze

* pid_kryt_output.bag zawiera wyniki z oscylacji wokol ladowiska dla wzmocnienia krytycznego k_kryt = 0.8 (os X i Y)

## Po dostosowaniu nastaw (Ziegler-Nichols)

* pid_P_output.bag zawiera wyniki z oscylacji wokol ladowiska dla regulatora P (Kp=0.4)
	- for P it works well on high altitudes and also on low but worse than PID
* pid_PI_output.bag zawiera wyniki z oscylacji wokol ladowiska dla regulatora P (Kp=0.36, Ki=0.094)
	- for PI it works well on low altitude but on high it becomes very unstable, but it's not smooth
* pid_PID_output.bag zawiera wyniki z oscylacji wokol ladowiska dla regulatora P (Kp=0.48, Ki=0.213, Kd=0.259)
	- for PID it works well on low altitude but on high it becomes very unstable 
