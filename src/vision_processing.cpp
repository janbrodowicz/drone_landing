#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
        VisionProcessingControll() : it(nh)
        {
            image_sub = it.subscribe("/iris/usb_cam/image_raw", 1, &VisionProcessingControll::image_cb, this);
            image_pub = it.advertise("vision_processing/output_img", 1);
            cv::namedWindow("Image window");

            ROS_INFO("VisionProcessing node started");
        }

        ~VisionProcessingControll()
        {
            cv::destroyWindow("Image window");
        }

        void image_cb(const sensor_msgs::ImageConstPtr& msg)
        {
            cv_bridge::CvImagePtr cv_ptr;

            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

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

            cv::Mat out_img = std::get<0>(ccl);

            // Showing the processed image
            cv::imshow("Image window", out_img);
            cv::waitKey(3);

            // Creating output message and publishing it
            sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", out_img).toImageMsg();
            image_pub.publish(msg_out);

        }

    private:
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;
        image_transport::Publisher image_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    VisionProcessingControll vp;
    ros::spin();
    return 0;
}
