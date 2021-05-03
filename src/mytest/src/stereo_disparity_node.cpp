#include <iostream>
#include <stdio.h>
#include <string>
#include "mytest/stereo_disparity.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "match");
    ros::NodeHandle nh;

    stereo_disparity st(nh);

    ros::spin();
}
