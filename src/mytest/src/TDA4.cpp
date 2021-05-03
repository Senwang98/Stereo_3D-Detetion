#include <sys/stat.h>
#include <stdio.h>
#include <dirent.h>
#include <iostream>
#include <string>
#include "mytest/TDA4.h"
#include "mytest/IiBasicDef.h"

#ifdef __unix
#define fopen_s(pFile, filename, mode) ((*(pFile)) = fopen((filename), (mode))) == NULL
#endif

// TDA::TDA(ros::NodeHandle& _nh)
// {
//     this->nh = _nh;
//     sub_left_img = new message_filters::Subscriber<Image>(this->nh, "/camera/rectified_left", 1);
//     sub_right_img = new message_filters::Subscriber<Image>(this->nh, "/camera/rectified_right", 1);
//     // image_transport::ImageTransport it(nh);
//     // disparity_pub = it.advertise("/camera/disparity", 100);
//     typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
//     message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_left_img, sub_right_img);
//     sync.registerCallback(boost::bind(&TDA::CamCallback, this, _1, _2));
//     ros::spin();
// }

TDA::TDA(ros::NodeHandle &_nh)
{
    nh = _nh;
    message_filters::Subscriber<Image> sub_left_img(nh, "/camera/rectified_left", 1);
    message_filters::Subscriber<Image> sub_right_img(nh, "/camera/rectified_right", 1);
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_left_img, sub_right_img);
    sync.registerCallback(boost::bind(&TDA::CamCallback, this, _1, _2));

    image_transport::ImageTransport it(nh);
    disparity_pub = it.advertise("/camera/disparity", 1);

    ros::spin();
}

TDA::~TDA() {}

void TDA::CamCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{
    cv_bridge::CvImagePtr left_cv_ptr;
    cv_bridge::CvImagePtr right_cv_ptr;
    try
    {
        left_cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
        right_cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat left_img = left_cv_ptr->image;
    cv::Mat right_img = right_cv_ptr->image;
    // cv::imshow("left", left_img);
    // cv::imshow("right", right_img);
    // cv::waitKey(0);

    // 读取视差图数据
    FILE *src_fp = NULL;
    int src_ret = 0;
    cv::Size parseImageSize;
    parseImageSize.width = II_IMAGE_WIDE;
    parseImageSize.height = II_IMAGE_HEIGHT;
    cv::Mat disparityMat = cv::Mat::zeros(parseImageSize, CV_16UC1);
    // std::cout << left_msg->header.seq << std::endl;
    int num = left_msg->header.seq % 3;
    string path = "/home/wangsen/test_image/disp/disp_s16_000000000" + std::to_string(num) + ".bin";
    const char *disparityFilePath = path.c_str();
    if (fopen_s(&src_fp, disparityFilePath, "rb") == 0)
    {
        src_ret = fread(disparityMat.data, parseImageSize.height * parseImageSize.width * sizeof(unsigned short), 1, src_fp);
        fclose(src_fp);
    }
    sensor_msgs::ImagePtr msg_disp = cv_bridge::CvImage(std_msgs::Header(), "mono16", disparityMat).toImageMsg();
    // msg_disp->header.stamp = left_msg->header.stamp;
    msg_disp->header.seq = left_msg->header.seq;
    this->disparity_pub.publish(msg_disp);
    cout << "Pic disp has been published!" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TDA4");
    ros::NodeHandle nh;
    TDA t(nh);
    ros::spin();
    return 0;
}