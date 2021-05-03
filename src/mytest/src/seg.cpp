#include <sys/stat.h>
#include <stdio.h>
#include <dirent.h>
#include "mytest/seg.h"
#include "mytest/IiBasicDef.h"

SEGMENT::SEGMENT(ros::NodeHandle &_nh)
{
    nh = _nh;
    image_transport::ImageTransport it(nh);
    this->seg_pub = it.advertise("/camera/segment", 100);
    this->sub_right_img = it.subscribe("/camera/rectified_right", 100, &SEGMENT::Callback, this);
    ros::spin();
}

SEGMENT::~SEGMENT() {}

void SEGMENT::Callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr seg_ptr;
    try
    {
        seg_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat seg = seg_ptr->image;
    // cv::imshow("seg", seg);
    // cv::waitKey(0);
    int i = 9;
    // int i = msg->header.seq % 9;
    // cv::Mat seg_result = cv::imread("/home/wangsen/test_image/seg/" + std::to_string(i) + ".bmp", 0);
    cv::Mat seg_result = cv::imread("/home/wangsen/data0420/seg/2/5/" + std::to_string(i) + ".bmp", 0);
    sensor_msgs::ImagePtr msg_seg = cv_bridge::CvImage(std_msgs::Header(), "mono8", seg_result).toImageMsg();
    // msg_seg->header.stamp = msg->header.stamp;
    msg_seg->header.seq = msg->header.seq;
    this->seg_pub.publish(msg_seg);
    std::cout << "Pic seg has been published!" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segment");
    ros::NodeHandle nh;
    SEGMENT t(nh);
    ros::spin();
    return 0;
}