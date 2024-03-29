#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros_landing/droneLand.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <rosbag/bag.h>
#include <tf/transform_listener.h>

#include <termios.h>
#include <Eigen3/Eigen/Dense>

#include "Regulators/PIDController.h"
#include "Regulators/KalmanFilter.h"
#include "Regulators/KalmanFilterTypes.h"


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


template<int State, int Input, int Measure>
class UavControl
{
    public:
        
        UavControl() : m_landing(false), m_ang(-500), time_now(ros::Time::now()), pid_x(1, 0, 0), pid_y(1, 0, 0), m_first_loop(true)
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            droneLand_sub = nh.subscribe<ros_landing::droneLand>("/droneLand", 10, &UavControl::droneLand_cb, this);
            lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, &UavControl::lidar_cb, this);
            drone_actual_pose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &UavControl::pose_cb, this);
            drone_velocity = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 10, &UavControl::velocity_cb, this);
            drone_accel = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data_raw", 10, &UavControl::accel_cb, this);
            landing_pad_pos = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, &UavControl::landing_pad_cb, this);

            cmd_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            PIDx = nh.advertise<std_msgs::Float32>("PID/x", 10);
            PIDy = nh.advertise<std_msgs::Float32>("PID/y", 10);

            // bag_pid.open("pid_output.bag", rosbag::bagmode::Write);
            // bag_kalman.open("kalman_output.bag", rosbag::bagmode::Write);
            bag_drone_pad_pos.open("drone_pad_pos.bag", rosbag::bagmode::Write);

            m_actual_pose.reserve(3);
            m_velocity.reserve(3);
            m_accel.reserve(3);
            m_landing_pad_pos.reserve(3);
        }

        UavControl(kalman::KalmanFilter<State, Input, Measure> filter) : m_landing(false), m_ang(-500), time_now(ros::Time::now()), pid_x(1, 0, 0), pid_y(1, 0, 0), m_kalmanFilter(filter), m_first_loop(true)
        {
            state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &UavControl::state_cb, this);
            droneLand_sub = nh.subscribe<ros_landing::droneLand>("/droneLand", 10, &UavControl::droneLand_cb, this);
            lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, &UavControl::lidar_cb, this);
            // drone_actual_pose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &UavControl::pose_cb, this);
            drone_velocity = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 10, &UavControl::velocity_cb, this);
            drone_accel = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data_raw", 10, &UavControl::accel_cb, this);
            landing_pad_pos = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, &UavControl::landing_pad_cb, this);

            cmd_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            PIDx = nh.advertise<std_msgs::Float32>("PID/x", 10);
            PIDy = nh.advertise<std_msgs::Float32>("PID/y", 10);

            x_measure_pub = nh.advertise<std_msgs::Float32>("x_measure", 10);
            y_measure_pub = nh.advertise<std_msgs::Float32>("y_measure", 10);

            bag_pid.open("pid_output.bag", rosbag::bagmode::Write);
            bag_kalman.open("kalman_output.bag", rosbag::bagmode::Write);
            bag_drone_pad_pos.open("drone_pad_pos.bag", rosbag::bagmode::Write);

            m_actual_pose.reserve(3);
            m_velocity.reserve(3);
            m_accel.reserve(3);
            m_landing_pad_pos.reserve(3);
        }

        ~UavControl()
        {
            bag_pid.close();
            bag_kalman.close();
            bag_drone_pad_pos.close();
        }

        // void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
        // {
        //     m_actual_pose[0] = msg->pose.position.x;
        //     m_actual_pose[1] = msg->pose.position.y;
        //     m_actual_pose[2] = msg->pose.position.z;
        // }

        void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
        {
            m_velocity[0] = msg->twist.linear.x;
            m_velocity[1] = msg->twist.linear.y;
            m_velocity[2] = msg->twist.linear.z;
        }

        void accel_cb(const sensor_msgs::Imu::ConstPtr& msg)
        {
            m_accel[0] = msg->linear_acceleration.x;
            m_accel[1] = msg->linear_acceleration.y;
            m_accel[2] = msg->linear_acceleration.z;
        }

        void landing_pad_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
        {
            m_landing_pad_pos[0] = msg->pose[1].position.x;
            m_landing_pad_pos[1] = msg->pose[1].position.y;
            m_landing_pad_pos[2] = msg->pose[1].position.z;

            m_actual_pose[0] = msg->pose[2].position.x;
            m_actual_pose[1] = msg->pose[2].position.y;
            m_actual_pose[2] = msg->pose[2].position.z;
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

                if(m_landing)
                {

                    //================================================================================
                    //================================================================================

                    // KALMAN FILTER OPERATIONS

                    //================================================================================
                    //================================================================================

                    position_in_cam = tf::Vector3(m_y, m_x, 0.2);

                    try
                    {
                        ros::Time now = ros::Time::now();
                        tf_listener.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));

                        tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), T_world_cam);
                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                    }

                    position_in_world = T_world_cam * position_in_cam;

                    if(m_first_loop)
                    {
                        Eigen::Matrix<double, 4, 1> init{{m_actual_pose[0]}, {m_actual_pose[1]}, {0.05}, {0.05}};
                        m_kalmanFilter.updateInitial(init);
                    }
                    
                    if(!m_first_loop)
                    {
                        // creating vector for measurement
                        Eigen::Matrix<double, 2, 1> measurement{{position_in_world.getX() - 1}, {position_in_world.getY()}};

                        // kalman update step
                        m_kalmanFilter.updateEstimate(measurement);
                    }

                    // update of A and Q matrices
                    m_kalmanFilter.update_AQmatrix(delta_t);

                    // setting input to the system to 0.1 (process noise)
                    // Eigen::Matrix<double, 6, 1> input{{0.1}, {0.01}, {0.01}, {0.01}, {0.01}, {0.01}};
                    Eigen::Matrix<double, 4, 1> input{{0.0001}, {0.0001}, {0.0001}, {0.0001}};
                    // Eigen::Matrix<double, 2, 1> input{{(-0.4) * m_y}, {(-0.4) * m_x}};
                    // Eigen::Matrix<double, 2, 1> input{{0.001}, {0.001}};

                    // kalman prediction step
                    m_kalman_out = m_kalmanFilter.predictEstimate(input);

                    try
                    {
                        ros::Time now = ros::Time::now();
                        tf_listener.waitForTransform("/base_link", "/map", now, ros::Duration(3.0));

                        tf_listener.lookupTransform("/base_link", "/map", now, T_world_cam_inv);
                    }
                    catch(tf::TransformException ex)
                    {
                        ROS_ERROR("%s",ex.what());
                        ros::Duration(1.0).sleep();
                    }

                    // T_world_cam_inv = T_world_cam;
                    // T_world_cam_inv.inverse();

                    position_in_world_kalman = tf::Vector3(m_kalman_out(0), m_kalman_out(1), 0.2);

                    position_in_cam_kalman = T_world_cam_inv * position_in_world_kalman;

                    //================================================================================
                    //================================================================================

                    // PID OPERATIONS

                    //================================================================================
                    //================================================================================

                    // double x_kalman_pid = position_in_cam_kalman.getX() * (-1);
                    // double y_kalman_pid = (position_in_cam_kalman.getY() - 1) * (-1);

                    // m_x_pid = pid_x.run(0, x_kalman_pid, delta_t);
                    // m_y_pid = pid_y.run(0, y_kalman_pid, delta_t);

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
                    x_estimate.data = m_kalman_out(0);
                    y_estimate.data = m_kalman_out(1);

                    // actual pose to ros msgs
                    std_msgs::Float32 x_actual;
                    std_msgs::Float32 y_actual;
                    std_msgs::Float32 z_actual;
                    x_actual.data = m_actual_pose[0];
                    y_actual.data = m_actual_pose[1];
                    z_actual.data = m_actual_pose[2];

                    // landing pad pose to ros msgs
                    std_msgs::Float32 x_pad;
                    std_msgs::Float32 y_pad;
                    std_msgs::Float32 z_pad;
                    x_pad.data = m_landing_pad_pos[0];
                    y_pad.data = m_landing_pad_pos[1];
                    z_pad.data = m_landing_pad_pos[2];

                    // measurement to ros msgs
                    std_msgs::Float32 x_measure;
                    std_msgs::Float32 y_measure;
                    x_measure.data = m_x;
                    y_measure.data = m_y;
                    // x_measure.data = position_in_world.getX() - 1;
                    // y_measure.data = position_in_world.getY();

                    std_msgs::Float32 x_measure_tf;
                    std_msgs::Float32 y_measure_tf;
                    x_measure_tf.data = position_in_cam_kalman.getX();
                    y_measure_tf.data = position_in_cam_kalman.getY();

                    std_msgs::Float32 x_global;
                    std_msgs::Float32 y_global;
                    x_global.data = position_in_world.getX() - 1;
                    y_global.data = position_in_world.getY();


                    // write data to a bag file
                    if(m_landing)
                    {
                        bag_pid.write("pid_x", ros::Time::now(), x_msg);
                        bag_pid.write("pid_y", ros::Time::now(), y_msg);

                        bag_kalman.write("estimate_x", ros::Time::now(), x_estimate);
                        bag_kalman.write("estimate_y", ros::Time::now(), y_estimate);
                        bag_kalman.write("measurement_x", ros::Time::now(), x_measure);
                        bag_kalman.write("measurement_y", ros::Time::now(), y_measure);
                        bag_kalman.write("measurement_tf_x", ros::Time::now(), x_measure_tf);
                        bag_kalman.write("measurement_tf_y", ros::Time::now(), y_measure_tf);
                        bag_kalman.write("global_x", ros::Time::now(), x_global);
                        bag_kalman.write("global_y", ros::Time::now(), y_global);
                        // bag_kalman.write("actual_x", ros::Time::now(), x_actual);
                        // bag_kalman.write("actual_y", ros::Time::now(), y_actual);

                        bag_drone_pad_pos.write("drone_x", ros::Time::now(), x_actual);
                        bag_drone_pad_pos.write("drone_y", ros::Time::now(), y_actual);
                        bag_drone_pad_pos.write("drone_z", ros::Time::now(), z_actual);

                        bag_drone_pad_pos.write("pad_x", ros::Time::now(), x_pad);
                        bag_drone_pad_pos.write("pad_y", ros::Time::now(), y_pad);
                        bag_drone_pad_pos.write("pad_z", ros::Time::now(), z_pad);
                    }

                    PIDx.publish(x_msg);
                    PIDy.publish(y_msg);

                    x_measure_pub.publish(x_measure);
                    y_measure_pub.publish(y_measure);

                    m_first_loop = false;
                }
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
        bool m_first_loop;
        double m_x_pid;
        double m_y_pid;
        double m_x;
        double m_y;
        double m_ang;
        double m_lidar;
        std::vector<double> m_actual_pose;
        std::vector<double> m_velocity;
        std::vector<double> m_accel;
        std::vector<double> m_landing_pad_pos;

        pid::PIDController pid_x;
        pid::PIDController pid_y;

        kalman::KalmanFilter<State, Input, Measure> m_kalmanFilter;
        Eigen::Matrix<double, State, 1> m_kalman_out;
    
    private:

        ros::NodeHandle nh;
        ros::Subscriber state_sub;
        ros::Subscriber droneLand_sub;
        ros::Subscriber lidar_sub;
        ros::Subscriber drone_actual_pose;
        ros::Subscriber drone_velocity;
        ros::Subscriber drone_accel;
        ros::Subscriber landing_pad_pos;
        ros::Publisher local_pos_pub;
        ros::Publisher cmd_vel_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        mavros_msgs::State current_state; 

        ros::Time time_now;

        ros::Publisher PIDx;
        ros::Publisher PIDy;

        ros::Publisher x_measure_pub;
        ros::Publisher y_measure_pub;

        rosbag::Bag bag_pid;
        rosbag::Bag bag_kalman;
        rosbag::Bag bag_drone_pad_pos;
        tf::TransformListener tf_listener;
        tf::StampedTransform T_world_cam;
        tf::StampedTransform T_world_cam_inv;
        tf::Vector3 position_in_cam, position_in_world;
        tf::Vector3 position_in_cam_kalman, position_in_world_kalman;
};
     

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    //================================================================================
    //================================================================================

    // KALMAN FILTER INITIALIZATION

    //================================================================================
    //================================================================================

    
    // Kalman Filter initialization
    kalman::KalmanFilter<4, 4, 2> kalmanFilter(kalman_CV::A, kalman_CV::B, kalman_CV::C,
                                               kalman_CV::G, kalman_CV::Q, kalman_CV::R,
                                               kalman_CV::w, kalman_CV::P0, kalman_CV::x0);


    //================================================================================
    //================================================================================

    // NODE OPERATIONS INITIALIZATION

    //================================================================================
    //================================================================================

    // UavControl initialization
    UavControl<4, 4, 2> uav(kalmanFilter);

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
                // if(!uav.m_landing)
                // {
                    pos.velocity.z += 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                // }  
				break;
			case KEYCODE_D:
                // if(!uav.m_landing)
                // {
                    pos.velocity.z -= 0.1;
                    ROS_INFO("X: %f, Y: %f, Z: %f, Yaw: %f", pos.velocity.x, pos.velocity.y, pos.velocity.z, pos.yaw_rate);
                // }
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
                    ROS_INFO("VELOCITIES RESET!!!");
                }
				break;
            case KEYCODE_C:
                // turn on automatic landing
                uav.m_landing = true;
                ROS_INFO("AUTOMATIC LANDING PROCEDURE SELECTED!!!");
                break;
            case KEYCODE_V:
                // turn off automatic landing also clear sum of errors and previous error for PIDs
                uav.m_landing = false;

                uav.pid_x.reset_pid();
                uav.pid_y.reset_pid();

                uav.m_kalmanFilter.resetKalman();
                uav.m_first_loop = true;

                ROS_INFO("AUTOMATIC LANDING PROCEDURE TUNRNED OFF!!!");
                break;
			default:
				break;				
		}
        
        // if automatic landing is turned on and landing pad is detected
        if(uav.m_landing && uav.m_ang != -500)
        {
            if(uav.m_lidar > 0.5)
            {
                uav.pid_x.update_pid_settings({0.45, 0.0, 0.0});
                uav.pid_y.update_pid_settings({0.45, 0.0, 0.0});
                // uav.pid_x.update_pid_settings({0.4, 0.005, 0.1});
                // uav.pid_y.update_pid_settings({0.4, 0.005, 0.1});

                pos.velocity.y = uav.m_x_pid;
                pos.velocity.x = uav.m_y_pid;

                if(std::sqrt(std::pow(uav.m_x, 2) - std::pow(uav.m_y, 2)) <= 0.15)
                {
                    pos.velocity.z = -0.07;
                }
                else
                {
                    pos.velocity.z = 0.0;
                }
            }
            else if(uav.m_lidar <= 0.5)
            {
                uav.pid_x.update_pid_settings({0.5, 0.01, 0.15});
                uav.pid_y.update_pid_settings({0.5, 0.01, 0.15});

                pos.velocity.y = uav.m_x_pid;
                pos.velocity.x = uav.m_y_pid;

                if(std::sqrt(std::pow(uav.m_x, 2) - std::pow(uav.m_y, 2)) <= 0.08)
                {
                    pos.velocity.z = -0.1;
                }
                else
                {
                    pos.velocity.z = 0.0;
                }
            }

            // pos.velocity.y = uav.m_x_pid;
            // pos.velocity.x = uav.m_y_pid;
        }

        // publish message with drone velocity
        uav.set_velocity(pos);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}