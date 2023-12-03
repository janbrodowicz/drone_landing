#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros_landing/droneLand.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "VisionProcessing/AltitudeDataProcessing.h"
#include "VisionProcessing/FrameProcessing.h"
#include "VisionProcessing/ObjectParameterAnalysis.h"


// -------------------------------------------------------------------------------------------------------------------
// CONSTANT PARAMETERS
// -------------------------------------------------------------------------------------------------------------------

#define M_PI 3.14159265358979323846

// The size of the input image
#define WIDTH 1280
#define HEIGHT 720

// The size of windows for thresholding
#define pSX 128
#define pSY 128


class VisionProcessingControll
{
    public:
        VisionProcessingControll() : sync(image_sub, lidar_sub, 100), it(nh)
        {
            droneLand_pub = nh.advertise<ros_landing::droneLand>("/droneLand", 10);

            image_sub.subscribe(nh, "/iris/usb_cam/image_raw", 1);
            lidar_sub.subscribe(nh, "/laser/scan", 1);

            sync.registerCallback(boost::bind(&VisionProcessingControll::callback, this, _1, _2));

            cv::namedWindow("Image window");

            ROS_INFO("VisionProcessing node started");
        }

        ~VisionProcessingControll()
        {
            cv::destroyWindow("Image window");
        }

        void callback(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::LaserScan::ConstPtr& lidar)
        {
            // -------------------------------------------------------------------------------------------------------------------
            // Converting image message to OpenCV format
            // -------------------------------------------------------------------------------------------------------------------

            cv_bridge::CvImagePtr cv_ptr;

            try
            {
                cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // -------------------------------------------------------------------------------------------------------------------
            // ALTITUDE DATA PROCESSING
            // -------------------------------------------------------------------------------------------------------------------

            // Load altitude measurement from lidar
            altitude::LidarAltitude altitude(lidar->ranges[0]);

            // Calculate expected circle size
            int circle_size = altitude.exp_circle_size();

            // Calculate expected square_size
            int square_size = altitude.exp_square_size(circle_size);

            // -------------------------------------------------------------------------------------------------------------------
            // CONSECUTIVE FRAMES PROCESSING
            // -------------------------------------------------------------------------------------------------------------------

            // Load image
            frame::FrameProcessing frame(cv_ptr->image, HEIGHT, WIDTH, pSX, pSY);

            // Conversion to grey and Gaussian blur
            cv::Mat grey = frame.grey_conversion_blur(5, 0.65);

            // Thresholding in windows
            std::tuple<cv::Mat, cv::Mat> windows_thr = frame.thresholding_in_windows();

            // Interpolation of image thresholded in windows
            cv::Mat IB_L = frame.interpolation(std::get<1>(windows_thr));

            // Dilation 3x3
            cv::Mat IB_D = frame.dilatation(IB_L, 3);

            // Median filter 5x5
            cv::Mat IB_M = frame.median_filter(IB_D, 5);

            // Erosion 3x3
            cv::Mat IB_E = frame.erosion(IB_M, 3);

            // Connected Component Labelling (CCL)
            std::tuple<cv::Mat, cv::Mat, cv::Mat> ccl = frame.connected_component_labelling(IB_E);

            // Convert binary image to 3-channel image
            cv::Mat IB_VIS = frame.bin_to_3ch(IB_E);

            // Extract stats from CCL
            cv::Mat stats = std::get<1>(ccl);

            // -------------------------------------------------------------------------------------------------------------------
            // OBJECT PARAMETER ANALYSIS
            // -------------------------------------------------------------------------------------------------------------------

            object::ObjectParameterAnalysis object(IB_VIS, stats, circle_size, square_size, HEIGHT, WIDTH);

            // Iterate through all detected objects without background
            std::vector<std::vector<std::vector<double>>> detection_vec= object.all_det_obj();

            // Iterate through all detected circles, squares and rectangles
            std::tuple<cv::Mat, std::vector<double>, double> iter_res = object.draw_angle_cent(detection_vec);

            // Calculate the distance from the centre of the line to the centre of the image (in pixels and centimetres)
            cv::Mat image_det = std::get<0>(iter_res);
            std::vector<double> centre = std::get<1>(iter_res);
            double angle = std::get<2>(iter_res);
            std::vector<std::vector<double>> dist_vec = object.calc_dist(centre);

            // Publish data for drone controll (with PID)
            ros_landing::droneLand msgDrone;
            msgDrone.X = dist_vec[1][0];
            msgDrone.Y = dist_vec[1][1];
            msgDrone.angle = angle;
            msgDrone.header.stamp = ros::Time::now();
            droneLand_pub.publish(msgDrone);

            // Showing the processed image
            cv::imshow("Image window", image_det);
            cv::waitKey(3);

            ROS_INFO("X distance [cm]: %f, Y distance [cm]: %f, Orientation angle [deg]: %f", dist_vec[1][0], dist_vec[1][1], angle);

        }

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        ros::Publisher droneLand_pub;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub;
        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::LaserScan> sync;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    VisionProcessingControll vp;
    ros::spin();
    return 0;
}