#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros_landing/droneLand.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>

#include <termios.h>
#include <Eigen3/Eigen/Dense>

#include "Regulators/PIDController.h"
#include "Regulators/KalmanFilter.h"


// definition of keybors keys hex codes for drone manual control
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



class UavControl
{
    public:
        
        UavControl() : m_landing(false), m_ang(-500), time_now(ros::Time::now()), pid_x(1, 0, 0), pid_y(1, 0, 0)
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            droneLand_sub = nh.subscribe<ros_landing::droneLand>("/droneLand", 10, &UavControl::droneLand_cb, this);
            lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, &UavControl::lidar_cb, this);
            drone_actual_pose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &UavControl::pose_cb, this);

            cmd_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            PIDx = nh.advertise<std_msgs::Float32>("PID/x", 10);
            PIDy = nh.advertise<std_msgs::Float32>("PID/y", 10);

            bag_pid.open("pid_output.bag", rosbag::bagmode::Write);
            bag_kalman.open("kalman_output.bag", rosbag::bagmode::Write);

            m_actual_pose.reserve(3);
        }

        UavControl(kalman::KalmanFilter filter) : m_landing(false), m_ang(-500), time_now(ros::Time::now()), pid_x(1, 0, 0), pid_y(1, 0, 0), kalmanFilter(filter)
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            droneLand_sub = nh.subscribe<ros_landing::droneLand>("/droneLand", 10, &UavControl::droneLand_cb, this);
            lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, &UavControl::lidar_cb, this);
            drone_actual_pose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &UavControl::pose_cb, this);

            cmd_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            PIDx = nh.advertise<std_msgs::Float32>("PID/x", 10);
            PIDy = nh.advertise<std_msgs::Float32>("PID/y", 10);

            bag_pid.open("pid_output.bag", rosbag::bagmode::Write);
            bag_kalman.open("kalman_output.bag", rosbag::bagmode::Write);

            m_actual_pose.reserve(3);
        }

        ~UavControl()
        {
            bag_pid.close();
            bag_kalman.close();
        }

        void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            m_actual_pose[0] = msg->pose.position.x;
            m_actual_pose[1] = msg->pose.position.y;
            m_actual_pose[2] = msg->pose.position.z;
        }

        void state_cb(const mavros_msgs::State::ConstPtr& msg)
        {
            current_state = *msg;
        }

        void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
        {
            m_lidar = msg->ranges[0];
        }

        void droneLand_cb(const ros_landing::droneLand::ConstPtr& msg)
        {

            m_ang = msg->angle;

            if(m_ang != -500.0)
            {
                double delta_t = (ros::Time::now() - time_now).toSec();

                // calculate from cm to m
                m_x = msg->X / 100.0;
                m_y = msg->Y / 100.0;

                //================================================================================
                //================================================================================

                // KALMAN FILTER OPERATIONS

                //================================================================================
                //================================================================================

                // update of A and Q matrices
                kalmanFilter.update_AQmatrix(delta_t);

                // create matrix for estimation output
                Eigen::Matrix<double, 4, 1> kalman_out;

                // setting input to the system to 0
                Eigen::Matrix<double, 4, 1> input; input.setZero();

                // kalman prediction step
                kalman_out = kalmanFilter.predictEstimate(input);

                // creating vector for measurement
                Eigen::Vector2d measurement(m_x, m_y);

                // kalman update step
                kalmanFilter.updateEstimate(measurement);

                //================================================================================
                //================================================================================

                // PID OPERATIONS

                //================================================================================
                //================================================================================

                m_x_pid = pid_x.run(0, m_x, delta_t);
                m_y_pid = pid_y.run(0, m_y, delta_t);

                // Publishing PID output for rqt to plot
                std_msgs::Float32 x_msg;
                std_msgs::Float32 y_msg;
                x_msg.data = m_x_pid;
                y_msg.data = m_y_pid;

                // kalman output to ros msgs
                std_msgs::Float32 x_estimate;
                std_msgs::Float32 y_estimate;
                x_estimate.data = kalman_out(0);
                y_estimate.data = kalman_out(1);

                // actual pose to ros msgs
                std_msgs::Float32 x_actual;
                std_msgs::Float32 y_actual;
                x_actual.data = m_actual_pose[0];
                y_actual.data = m_actual_pose[1];

                // measurement to ros msgs
                std_msgs::Float32 x_measure;
                std_msgs::Float32 y_measure;
                x_measure.data = m_x;
                y_measure.data = m_y;


                // write data to a bag file
                if(m_landing)
                {
                    bag_pid.write("pid_x", ros::Time::now(), x_msg);
                    bag_pid.write("pid_y", ros::Time::now(), y_msg);

                    bag_kalman.write("estimate_x", ros::Time::now(), x_estimate);
                    bag_kalman.write("estimate_y", ros::Time::now(), y_estimate);
                    bag_kalman.write("measurement_x", ros::Time::now(), x_measure);
                    bag_kalman.write("measurement_y", ros::Time::now(), y_measure);
                    bag_kalman.write("actual_x", ros::Time::now(), x_actual);
                    bag_kalman.write("actual_y", ros::Time::now(), y_actual);
                }

                PIDx.publish(x_msg);
                PIDy.publish(y_msg);
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
        double m_x_pid;
        double m_y_pid;
        double m_x;
        double m_y;
        double m_ang;
        double m_lidar;
        std::vector<double> m_actual_pose;

        pid::PIDController pid_x;
        pid::PIDController pid_y;
    
    private:

        ros::NodeHandle nh;
        ros::Subscriber state_sub;
        ros::Subscriber droneLand_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber drone_actual_pose;
        ros::Publisher local_pos_pub;
        ros::Publisher cmd_vel_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state; 

        ros::Time time_now;

        ros::Publisher PIDx;
        ros::Publisher PIDy;

        rosbag::Bag bag_pid;
        rosbag::Bag bag_kalman;

        kalman::KalmanFilter kalmanFilter;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //================================================================================
    //================================================================================

    // KALMAN FILTER INITIALIZATION

    //================================================================================
    //================================================================================

    Eigen::Matrix<double, 4, 4> A{{1, 0, 0.5, 0},
                                  {0, 1, 0, 0.5},
                                  {0, 0, 1, 0},
                                  {0, 0, 0, 1}};

    // control matrix
    Eigen::Matrix<double, 4, 4> B; B.setZero();

    // observation matrix
    Eigen::Matrix<double, 2, 4> C{{1, 0, 0, 0},
                                  {0, 1, 0, 0}};

    // process noise vector
    Eigen::Vector2d w(0.01, 0.01); 

    // first assupmption is that sampling time is 0.5s (during operation it'll be updated)
    Eigen::Matrix<double, 4, 4> Q{{0.0156 * w(0), 0, 0.0416 * w(0), 0},
                                  {0, 0.0156 * w(1), 0, 0.0416 * w(1)},
                                  {0.0416 * w(0), 0, 0.25 * w(0), 0},
                                  {0, 0.0416 * w(1), 0, 0.25 * w(1)}}; 
    
    // measurement covariance matrix (measurement noise)
    Eigen::Matrix<double, 2, 2> R{{0.0001, 0},
                                  {0, 0.0001}}; 

    // initial estimate covariance (guess)
    Eigen::Matrix<double, 4, 4> P0{{0.1, 0, 0, 0},
                                   {0, 0.1, 0, 0},
                                   {0, 0, 0.1, 0},
                                   {0, 0, 0, 0.1}}; // TODO: find out how to calculate this matrix

    // initial state vector (guess)
    Eigen::Matrix<double, 4, 1> x0{{0}, {0}, {0.01}, {0.01}};

    // Kalman Filter initialization
    kalman::KalmanFilter kalmanFilter(A, B, C, Q, R, w, P0, x0);


    //================================================================================
    //================================================================================

    // NODE OPERATIONS INITIALIZATION

    //================================================================================
    //================================================================================

    // UavControl initialization
    UavControl uav(kalmanFilter);

    // PID parameters
    std::vector<double> PidParams = {1, 0, 0};

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

    // set OFFBOARD mode to controll drone from PC (message)
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // arm the drone (message)
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        // set OFFBOARD and ARM the UAV
        if( uav.get_curr_state().mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( uav.set_mode(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        } 
        else 
        {
            if( !uav.get_curr_state().armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( uav.arm_uav(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();
            }
        }

        // get PID parameters
        if(ros::param::get("/pidParams", PidParams) && uav.pid_x.get_params() != PidParams)
        {
            uav.pid_x.update_pid_settings(PidParams);
            uav.pid_y.update_pid_settings(PidParams);
            ROS_INFO("PID parameters updated: Kp: %f, Ki: %f, Kd: %f", PidParams[0], PidParams[1], PidParams[2]);
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
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;
			case KEYCODE_L:
                if(!uav.m_landing)
                {
                    pos.velocity.y -= 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;
			case KEYCODE_J:
                if(!uav.m_landing)
                {
                    pos.velocity.y += 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;
			case KEYCODE_K:
                if(!uav.m_landing)
                {
                    pos.velocity.x -= 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;
			case KEYCODE_U:
                if(!uav.m_landing)
                {
                    pos.velocity.z += 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }  
				break;
			case KEYCODE_D:
                if(!uav.m_landing)
                {
                    pos.velocity.z -= 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;	
            case KEYCODE_Z:
                if(!uav.m_landing)
                {
                    pos.yaw_rate += 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                }
				break;
            case KEYCODE_X:
                if(!uav.m_landing)
                {
                    pos.yaw_rate -= 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
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
                // turn on automatic landing
                uav.m_landing = true;
                break;
            case KEYCODE_V:
                // turn off automatic landing also clear sum of errors and previous error for PIDs
                uav.m_landing = false;
                uav.pid_x.m_sum = 0;
                uav.pid_y.m_sum = 0;
                uav.pid_x.m_prev = 0;
                uav.pid_y.m_prev = 0;
                break;
			default:
				break;				
		}
        
        // if automatic landing is turned on and landing pad is detected
        if(uav.m_landing && uav.m_ang != -500)
        {
            // X and Y are different for drone coordinate frame and image frame
            pos.velocity.y = uav.m_x_pid;
            pos.velocity.x = uav.m_y_pid;
            pos.velocity.z = 0;

            // if distances on X and Y from center of the landing pad is less than 10cm and altitude is more than 1cm
            // if(std::abs(uav.m_x) <= 0.1 && std::abs(uav.m_y) <= 0.1 && uav.m_lidar > 0.1)
            // {
            //     // lower the altitude (by velocity)
            //     pos.velocity.z = -0.1;
            // }

            ROS_INFO("PID output: X: %f, Y: %f", uav.m_x, uav.m_y);
        }

        // publish message with drone velocity
        uav.set_velocity(pos);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}