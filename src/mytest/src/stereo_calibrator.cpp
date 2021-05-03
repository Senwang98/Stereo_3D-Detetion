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

#include "mytest/IiBasicDef.h"
#include "mytest/IIStereoCalibrator.h"

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rectify");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Publisher rectified_left_pub = it.advertise("/camera/rectified_left", 100);
	image_transport::Publisher rectified_right_pub = it.advertise("/camera/rectified_right", 100);

	ros::Rate loop_rate(1); //1HZ
	int i = 9;
	while (nh.ok())
	{
		// i = i % 9;
		string left_path = "/home/wangsen/data0420/2/5/left_image/" + to_string(i) + ".bmp";
		string right_path = "/home/wangsen/data0420/2/5/right_image/" + to_string(i) + ".bmp";

		cv::Mat left_img = cv::imread(left_path, 1);
		cv::Mat right_img = cv::imread(right_path, 1);

		//load the rectifed yml
		cv::Size m_size, expectedSize;
		m_size.width = II_IMAGE_WIDE;
		m_size.height = II_IMAGE_HEIGHT;
		expectedSize.width = m_size.width;
		expectedSize.height = m_size.height;

		CIIStereoCalibrator testCalibrator;
		testCalibrator.init("/home/wangsen/ros_test/src/mytest/stereo_calib_pointgrey.yml", expectedSize, 1.1, 1.1);
		// testCalibrator.init("/home/wangsen/ros_test/src/mytest/stereo_calib_stereo.yml", expectedSize, 1.5, 1.5);

		cv::Mat rectified_left, rectified_right;

		double rectifyTime = (double)cvGetTickCount();
		// testCalibrator.rectifyImages(selected_left, selected_right, rectified_left, rectified_right);
		testCalibrator.rectifyImages(left_img, right_img, rectified_left, rectified_right);
		rectifyTime = (double)cvGetTickCount() - rectifyTime;
		std::cout << "rectfy image time : " << rectifyTime / ((double)cvGetTickFrequency() * 1000) << std::endl;
		cv::imwrite("/home/wangsen/桌面/left.bmp",rectified_left);
		cv::imwrite("/home/wangsen/桌面/right.bmp",rectified_right);

		// cv::imshow("rectify_left",rectified_left);
		// cv::imshow("rectify_right",rectified_right);
		// cv::waitKey(0);
		//cout<<rectified_left<<endl;

		sensor_msgs::ImagePtr rec_left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectified_left).toImageMsg();
		sensor_msgs::ImagePtr rec_right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectified_right).toImageMsg();

		// ros::Time begin = ros::Time::now();
		// rec_left_msg->header.stamp = begin;
		// rec_right_msg->header.stamp = begin;
		rec_left_msg->header.seq = i;
		rec_right_msg->header.seq = i;
		rectified_left_pub.publish(rec_left_msg);
		rectified_right_pub.publish(rec_right_msg);
		ros::spinOnce();
		loop_rate.sleep();
		// i++;
	}

	// cv::imwrite("/home/wangsen/data0420/4/20/rect_left/"+to_string(num)+".bmp",rectified_left);
	// cv::imwrite("/home/wangsen/data0420/4/20/rect_right/"+to_string(num)+".bmp",rectified_right);
	return 0;
}

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "rectify");
// 	ros::NodeHandle nh;
// 	image_transport::ImageTransport it(nh);

// 	image_transport::Publisher rectified_left_pub = it.advertise("/camera/rectified_left", 100);
// 	image_transport::Publisher rectified_right_pub = it.advertise("/camera/rectified_right", 100);

// 	for (int i = 0; i <= 9; i++)
// 	{
// 		string left_path = "/home/wangsen/data0420/3/30/left_image/" + to_string(i) + ".bmp";
// 		string right_path = "/home/wangsen/data0420/3/30/right_image/" + to_string(i) + ".bmp";

// 		cv::Mat left_img = cv::imread(left_path, 1);
// 		cv::Mat right_img = cv::imread(right_path, 1);

// 		cv::Size m_size, expectedSize;
// 		m_size.width = II_IMAGE_WIDE;
// 		m_size.height = II_IMAGE_HEIGHT;
// 		expectedSize.width = m_size.width;
// 		expectedSize.height = m_size.height;

// 		CIIStereoCalibrator testCalibrator;
// 		testCalibrator.init("/home/wangsen/ros_test/src/mytest/stereo_calib_pointgrey.yml", expectedSize, 1.1, 1.1);

// 		cv::Mat rectified_left, rectified_right;

// 		double rectifyTime = (double)cvGetTickCount();
// 		testCalibrator.rectifyImages(left_img, right_img, rectified_left, rectified_right);
// 		rectifyTime = (double)cvGetTickCount() - rectifyTime;
// 		std::cout << "rectfy image time : " << rectifyTime / ((double)cvGetTickFrequency() * 1000) << std::endl;

// 		cv::imwrite("/home/wangsen/data0420/3/30/rect_left/" + to_string(i) + ".bmp", rectified_left);
// 		cv::imwrite("/home/wangsen/data0420/3/30/rect_right/" + to_string(i) + ".bmp", rectified_right);
// 		// cv::imwrite("/home/wangsen/data0420/4/20/rect_left/" + to_string(i) + ".bmp", rectified_left);
// 		// cv::imwrite("/home/wangsen/data0420/4/20/rect_right/" + to_string(i) + ".bmp", rectified_right);
// 	}
// 	return 0;
// }