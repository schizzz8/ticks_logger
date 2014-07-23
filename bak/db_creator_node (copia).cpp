#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

namespace enc = sensor_msgs::image_encodings;
std::ofstream my_log;
static const char WINDOW[] = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/rgb/image_rect", 1, &ImageConverter::imageCb, this);

        cv::namedWindow(WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            ros::Time ts = msg->header.stamp;
            std::cout << ts.toNSec() << "\n";
            std::cout << msg->encoding.c_str() << "\n";
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO8);
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        /*
        double minVal, maxVal;
        cv::Mat blur_img;
        minMaxLoc(cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
        cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
        cv::imshow(WINDOW, blur_img);
        */
        cv::imshow(WINDOW, cv_ptr->image);

        cv::waitKey(3);

    }
};

int main(int argc, char** argv)
{
    std::cout << "Esisto!!!\n";
    my_log.open("rgb_log.txt", std::ios::out );
    my_log << "RGB Image Timestamps: \n";
    ros::init(argc, argv, "db_creator_node");
    ImageConverter ic;
    ros::spin();
    my_log.close();
    return 0;
}
