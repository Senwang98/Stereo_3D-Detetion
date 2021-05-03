#ifndef SEG_H
#define SEG_H

#include <iostream>
#include <stdio.h>
#include <string>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>  
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sys/types.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "opencv/cxcore.h" 
#include <ctime>
#include <time.h>

#include "IiBasicDef.h"

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

class SEGMENT
{
public:
    SEGMENT(ros::NodeHandle& _nh);
    ~SEGMENT();  
    void Callback(const sensor_msgs::ImageConstPtr &msg);

private:
    ros::NodeHandle nh;
    image_transport::Subscriber sub_right_img;
    image_transport::Publisher seg_pub;
};

#endif