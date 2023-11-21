/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <termios.h>

#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_L 0x6c
#define KEYCODE_K 0x6b
#define KEYCODE_U 0x75
#define KEYCODE_D 0x64


char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	// else if(rv == 0)
	// 	ROS_INFO("no_key_pressed");
	else if(rv != 0)
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}


class UavControl
{
    public:

        UavControl()
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        }

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

        void set_pose(geometry_msgs::PoseStamped pose)
        {
            local_pos_pub.publish(pose);
        }

        bool set_mode(mavros_msgs::SetMode& mode)
        {
            bool ret = set_mode_client.call(mode);
            return ret;
        }

        bool arm_uav(mavros_msgs::CommandBool& arm_cmd)
        {
            bool ret = arming_client.call(arm_cmd);
            return ret;
        }

        mavros_msgs::State get_curr_state()
        {
            return current_state;
        }
    
    private:

        ros::NodeHandle nh;
        ros::Subscriber state_sub;
        ros::Publisher local_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    UavControl uav;

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !uav.get_curr_state().connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        uav.set_pose(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        // set OFFBOARD and ARM the UAV
        if( uav.get_curr_state().mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( uav.set_mode(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !uav.get_curr_state().armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( uav.arm_uav(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // get input from keyboard with out enter
        char c = 0;
		c = getch();

		switch(c)
		{
			case KEYCODE_I:
                pose.pose.position.x += 0.1;
				break;
			case KEYCODE_L:
                pose.pose.position.y -= 0.1;
				break;
			case KEYCODE_J:
                pose.pose.position.y += 0.1;
				break;
			case KEYCODE_K:
                pose.pose.position.x -= 0.1;
				break;
			case KEYCODE_U:
                pose.pose.position.z += 0.1;
				break;
			case KEYCODE_D:
                pose.pose.position.z -= 0.1;
				break;	
			default:
				break;				
		}

        // set pose of the UAV (carthesian)
        uav.set_pose(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}