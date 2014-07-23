#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>
#include <capygroovy/Ticks.h>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

void ticksCallback(const capygroovy::TicksConstPtr& msg)
{

}

void callback(const ImageConstPtr& msg,const ImageConstPtr& msg2)
{

    cv_bridge::CvImagePtr rgb_cv_ptr,dpt_cv_ptr;
    try
    {
        rgb_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        dpt_cv_ptr = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    double minVal, maxVal;
    cv::Mat blur_img;
    minMaxLoc(dpt_cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
    dpt_cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
    cv::imshow("RGB Image window", rgb_cv_ptr->image);
    cv::waitKey(3);
    //cv::imshow("DEPTH Image window", blur_img);
    //cv::waitKey(3);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_creator_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<Image> rgb_sub(nh,"/camera/rgb/image_rect",1);
    message_filters::Subscriber<Image> dpt_sub(nh,"/camera/depth/image_rect_raw",1);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dpt_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    message_filters::Subscriber<capygroovy::Ticks> ticks_sub(nh,"/ticks",1);
    message_filters::Cache<capygroovy::Ticks> ticks_cache(ticks_sub,10);
    ticks_cache.registerCallback(boost::bind(&ticksCallback,_1));

    cv::namedWindow("RGB Image window");
    //cv::namedWindow("DEPTH Image Window");
    cv::startWindowThread();
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
