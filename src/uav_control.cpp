#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros_landing/droneLand.h>

#include <termios.h>


#define KEYCODE_I 0x69
#define KEYCODE_J 0x6a
#define KEYCODE_L 0x6c
#define KEYCODE_K 0x6b
#define KEYCODE_U 0x75
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7a
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_Q 0x71


class PIDController
{
    public:

        PIDController(double Kp, double Ki, double Kd) : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd)
                                                        , m_sum(0), m_prev(0) {}

        double run(double set_point, double pv, double delta_t)
        {
            double err = set_point - pv;

            m_sum += err;

            double x_p = m_Kp * err + m_Ki * (m_sum) * delta_t + m_Kd * (err - m_prev) / delta_t;

            m_prev = err;

            return x_p;
        }
    
    private:

        double m_Kp;
        double m_Ki;
        double m_Kd;
        double m_sum;
        double m_prev;
};


class UavControl
{
    public:
        
        // TODO: Chose right Kp, Ki and Kd for PIDs
        UavControl() : m_landing(false), m_ang(-500), time_now(ros::Time::now()), pid_x(1, 0.1, 0.5), pid_y(1, 0.1, 0.5)
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            droneLand_sub = nh.subscribe<ros_landing::droneLand>("/droneLand", 10, &UavControl::droneLand_cb, this);
            cmd_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        }

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

        void droneLand_cb(const ros_landing::droneLand::ConstPtr& msg)
        {

            m_ang = msg->angle;

            if(m_ang != -500.0)
            {
                double delta_t = (ros::Time::now() - time_now).toSec();

                // TODO: figure out how to scale properly
                m_x = pid_x.run(0, msg->X, delta_t) / 700.0;
                m_y = pid_y.run(0, msg->Y, delta_t) / 700.0;
            }
            
            time_now = ros::Time::now();
        }

        void set_velocity(mavros_msgs::PositionTarget pos)
        {
            cmd_vel_pub.publish(pos);
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

        bool m_landing;
        double m_x;
        double m_y;
        double m_ang;
    
    private:

        ros::NodeHandle nh;
        ros::Subscriber state_sub;
        ros::Subscriber droneLand_sub;
        ros::Publisher local_pos_pub;
        ros::Publisher cmd_vel_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state; 

        PIDController pid_x;
        PIDController pid_y;
        ros::Time time_now;
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

    // set drone velocity
    mavros_msgs::PositionTarget pos;
    pos.coordinate_frame = 8;  // drone body coordinate frame
    pos.velocity.x = 0;
    pos.velocity.y = 0;
    pos.velocity.z = 0;
    pos.yaw_rate = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        uav.set_velocity(pos);
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
		c = uav.getch();

        switch(c)
		{
			case KEYCODE_I:
                if(!uav.m_landing)
                {
                    pos.velocity.x += 0.1;
                }
				break;
			case KEYCODE_L:
                if(!uav.m_landing)
                {
                    pos.velocity.y -= 0.1;
                }
				break;
			case KEYCODE_J:
                if(!uav.m_landing)
                {
                    pos.velocity.y += 0.1;
                }
				break;
			case KEYCODE_K:
                if(!uav.m_landing)
                {
                    pos.velocity.x -= 0.1;
                }
				break;
			case KEYCODE_U:
                if(!uav.m_landing)
                {
                    pos.velocity.z += 0.1;
                }  
				break;
			case KEYCODE_D:
                if(!uav.m_landing)
                {
                    pos.velocity.z -= 0.1;
                }
				break;	
            case KEYCODE_Z:
                if(!uav.m_landing)
                {
                    pos.yaw_rate += 0.1;
                }
				break;
            case KEYCODE_X:
                if(!uav.m_landing)
                {
                    pos.yaw_rate -= 0.1;
                }
				break;
            case KEYCODE_Q:
                if(!uav.m_landing)
                {
                    pos.velocity.x = 0;
                    pos.velocity.y = 0;
                    pos.velocity.z = 0;
                }
				break;
            case KEYCODE_C:
                uav.m_landing = true;
                break;
            case KEYCODE_V:
                uav.m_landing = false;
                break;
			default:
				break;				
		}

        if(uav.m_landing && uav.m_ang != -500)
        {
            // X and Y are different for drone coordinate frame and image frame
            pos.velocity.y = uav.m_x;
            pos.velocity.x = uav.m_y;
            pos.velocity.z = 0;

            ROS_INFO("PID output: X: %f, Y: %f", uav.m_x, uav.m_y);
        }

        // publish message with drone velocity
        uav.set_velocity(pos);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}