# Landing of a drone with MAVROS, ROS, PX4, Gazebo

### Steps before launching

1. Install ROS1 Noetic with Gazebo (full build): https://wiki.ros.org/noetic/Installation/Ubuntu
2. Download OpenCv (C++) preferably 4.2.0 
3. Download and build PX4 framework: https://docs.px4.io/main/en/dev_setup/building_px4.html
4. Download and build MAVROS: https://docs.px4.io/main/en/ros/mavros_installation.html
5. In PX4 framework find models directory -> lidar -> model.sdf and add the following under `</plugin>`:
	```
	<plugin name="lidar_node" filename="libgazebo_ros_laser.so">
          <robotNamespace></robotNamespace>
          <topicName>/laser/scan</topicName>
          <frameName>/lidar_sensor_link</frameName>
        </plugin>
        ```

### Launching steps

1. Go to ~/workspace_dir and build the workspace; command: "catkin build"
2. Go to ~/workspace_dir/src/ros_landing/ and run eviroment_vars.sh; command: ". ./enviroment_vars.sh" (open the file 	before and check if the paths match with yours)
3. Run the siumulation; command: "roslaunch ros_landing ros_landing_launch.launch"
