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
#include <unistd.h>
#include <iostream>
#include <fstream>

using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image, capygroovy::Ticks> MySyncPolicy;

std::vector<boost::shared_ptr<capygroovy::Ticks const> > ticks;
ros::Time last;

char dir [] = "/home/dede/image_dir/";
char rgbPrefix []= "rgb_";
char depthPrefix []= "depth_";
char jpgSuffix []= ".jpg";
char pgmSuffix []= ".pgm";

class DBCreator
{
    int count_;
    ros::NodeHandle nh_;
    message_filters::Subscriber<capygroovy::Ticks> ticks_sub_;
    message_filters::Cache<capygroovy::Ticks> ticks_cache_;

    message_filters::Subscriber<Image> rgb_sub_,dpt_sub_;
    Synchronizer<MySyncPolicy> sync_;

    std::ofstream db_;

public:

    DBCreator() : ticks_sub_(nh_,"/ticks",1),ticks_cache_(ticks_sub_,100),
        rgb_sub_(nh_,"/camera/rgb/image_rect",1),dpt_sub_(nh_,"/camera/depth/image_rect_raw",1),
        sync_(MySyncPolicy(10), rgb_sub_, dpt_sub_, ticks_sub_)
    {
        last = ros::Time::now();
        count_ = 0;
        ticks_sub_.registerCallback(&DBCreator::ticksCallback,this);
        sync_.registerCallback(boost::bind(&DBCreator::fileCallback,this, _1, _2, _3));

        db_.open("db_log.txt");
        db_ << "%%-- TIMESTAMP, Left Ticks, Right Ticks, Path to RGB, Path to Depth \n";
    }

    ~DBCreator()
    {
        db_.close();
    }

    void ticksCallback(const capygroovy::TicksConstPtr& msg){

    }

    void fileCallback(const ImageConstPtr& msg,const ImageConstPtr& msg2,const capygroovy::TicksConstPtr& msg3)
    {
        cv_bridge::CvImagePtr rgb_cv_ptr,dpt_cv_ptr;
        char buffer1[1024],buffer2[1024];

        ticks = ticks_cache_.getInterval(last,msg3->header.stamp);
        last = msg3->header.stamp;

        int left = 0,right = 0;
        for(std::vector<boost::shared_ptr<capygroovy::Ticks const> >::iterator it = ticks.begin(); it != ticks.end();++it){
            boost::shared_ptr<capygroovy::Ticks const> tmp = *it;
            left += tmp->leftTick;
            right += tmp->rightTick;
        }
        ticks.clear();

        try
        {
            rgb_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            dpt_cv_ptr = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_32FC1);
//		dpt_cv_ptr = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::TYPE_16UC1);


            double minVal, maxVal;
            cv::Mat blur_img;
            minMaxLoc(dpt_cv_ptr->image, &minVal, &maxVal); //find minimum and maximum intensities
            dpt_cv_ptr->image.convertTo(blur_img, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

            sprintf(buffer1,"%s%s%d%s", dir, rgbPrefix, count_, jpgSuffix);
            sprintf(buffer2,"%s%s%d%s", dir, depthPrefix, count_, pgmSuffix);
            count_++;

            std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PXM_BINARY);
            compression_params.push_back(0);

            cv::imwrite(buffer1,rgb_cv_ptr->image);
            cv::imwrite(buffer2,blur_img,compression_params);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        db_ << msg->header.stamp.toNSec() << "," << left << "," << right << "," << buffer1 << "," << buffer2 << "\n";

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_creator_node");
    DBCreator dbc;
    ros::spin();
    return 0;
}
