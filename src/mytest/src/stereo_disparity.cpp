#include "mytest/stereo_disparity.h"
#include <sys/stat.h>
#include <stdio.h>
#include <dirent.h>

stereo_disparity::stereo_disparity(ros::NodeHandle &_nh)
{
	nh = _nh;

	// sub_left_img = new message_filters::Subscriber<Image>(nh, "/camera/left/image_color", 1);
	// sub_right_img = new message_filters::Subscriber<Image>(nh, "/camera/right/image_color", 1);

	message_filters::Subscriber<Image> sub_left_img(nh, "/camera/rectified_left", 100);
	message_filters::Subscriber<Image> sub_right_img(nh, "/camera/rectified_right", 100);
	typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_left_img, sub_right_img);
	sync.registerCallback(boost::bind(&stereo_disparity::CamCallback, this, _1, _2));

	image_transport::ImageTransport it(nh);
	disparity_pub = it.advertise("/camera/disparity", 100);

	ros::spin();
}

stereo_disparity::~stereo_disparity()
{
	// cv::destroyAllWindows();
}

int buttonDownTimes = 0;
cv::Point2d points[2];
float old_Xc = 0, old_Yc = 0, old_Zc = 0;

void stereo_disparity::my_mouse_callback(int event, int x, int y, int flag, void *param)
{
	cv::Mat *depthImage = (cv::Mat *)param;

	int height = depthImage->rows;
	int width = depthImage->cols;

	if (height == 0 || width == 0)
		return;

	float *p = (float *)depthImage->data;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		float depth = *(p + y * width + x);

		points[buttonDownTimes].x = x;
		points[buttonDownTimes].y = y;

		buttonDownTimes++;
		buttonDownTimes %= 2;

		printf("Click on x = %d, y = %d,  depth value = %f mm.\n", x, y, depth);
		float Xc = 0, Yc = 0, Zc = depth;

		Xc = (x - II_CENTER_U) * depth / II_Focus;
		Yc = (y - II_CENTER_V) * depth / II_Focus;

		printf("Click on Xc = %f, Yc = %f,  Zc = %f mm.\n", Xc, Yc, Zc);

		if (buttonDownTimes == 0)
		{
			float fDistance = sqrt((Xc - old_Xc) * (Xc - old_Xc) + (Yc - old_Yc) * (Yc - old_Yc) + (Zc - old_Zc) * (Zc - old_Zc));
			printf("the distance of two points are %f\n", fDistance);
		}
		old_Xc = Xc, old_Yc = Yc, old_Zc = Zc;
	}
}

void stereo_disparity::CamCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
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

	//opencv dense disparity calculator
	CIIOpencvDisparityCalculator opencvDisparityCalulator;
	opencvDisparityCalulator.init();

	cv::Mat opencvDisparityMat, opencvDepthMat;
	cv::Mat *pOpencvDisparityMatDisplay = NULL;

	double opencvDisparityCaculationTime = (double)cvGetTickCount();
	opencvDisparityCalulator.calculateDisparity(left_img, right_img, opencvDisparityMat, opencvDepthMat);
	pOpencvDisparityMatDisplay = opencvDisparityCalulator.showColorDisparity();
	opencvDisparityCaculationTime = (double)cvGetTickCount() - opencvDisparityCaculationTime;
	std::cout << " Opencv calculate disparity time : " << opencvDisparityCaculationTime / ((double)cvGetTickFrequency() * 1000) << std::endl;

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", opencvDisparityMat).toImageMsg();
	// msg->header = right_msg->header; //based on the right images
	msg->header.seq = right_msg->header.seq;
	disparity_pub.publish(msg);

	// opencvDisparityMat.convertTo(opencvDisparityMat, CV_32F, 1.0 / 16);								   //除以16得到真实视差值
	// cv::Mat opencvDisparityMat8U = cv::Mat(opencvDisparityMat.rows, opencvDisparityMat.cols, CV_8UC1); //显示
	// normalize(opencvDisparityMat, opencvDisparityMat8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
	// cv::imshow("Disparity_Color", opencvDisparityMat8U);
	// cv::waitKey(0);

	// int x = 310, y = 375, w = 360, h = 150, gt = 4500;
	// // int x = 450, y = 295, w = 180, h = 70, gt = 9400;
	// // int x = 490, y = 260, w = 90, h = 40, gt = 21000;

	// int a1 = 0, a2 = 0, a3 = 0, a4 = 0, num = 0;
	// double sum = 0, avg = 0, ssum = 0, standard = 0;

	// for (int i = 0; i < w; i++)
	// {
	// 	for (int j = 0; j < h; j++)
	// 	{
	// 		if (opencvDepthMat.at<float>(y + j, x + i) > 0 && opencvDepthMat.at<float>(y + j, x + i) < 1000000)
	// 		{
	// 			num++;
	// 			sum += opencvDepthMat.at<float>(y + j, x + i);
	// 		}
	// 	}
	// }
	// avg = sum / num;

	// for (int i = 0; i < w; i++)
	// {
	// 	for (int j = 0; j < h; j++)
	// 	{
	// 		if (opencvDepthMat.at<float>(y + j, x + i) > 0 && opencvDepthMat.at<float>(y + j, x + i) < 1000000)
	// 		{
	// 			ssum += pow((opencvDepthMat.at<float>(y + j, x + i) - avg), 2);
	// 		}
	// 	}
	// }
	// standard = std::sqrt(ssum / (num - 1));

	// num = 0, sum = 0;
	// for (int i = 0; i < w; i++)
	// {
	// 	for (int j = 0; j < h; j++)
	// 	{
	// 		if (opencvDepthMat.at<float>(y + j, x + i) > 0 && opencvDepthMat.at<float>(y + j, x + i) > (avg - 3 * standard) && opencvDepthMat.at<float>(y + j, x + i) < (avg + 3 * standard))
	// 		{
	// 			if (fabs(opencvDepthMat.at<float>(y + j, x + i) - gt) / gt <= 0.1)
	// 				a1++;
	// 			else if (fabs(opencvDepthMat.at<float>(y + j, x + i) - gt) / gt <= 0.2)
	// 				a2++;
	// 			else if (fabs(opencvDepthMat.at<float>(y + j, x + i) - gt) / gt <= 0.3)
	// 				a3++;
	// 			else
	// 				a4++;
	// 			num++;
	// 			sum += opencvDepthMat.at<float>(y + j, x + i);
	// 		}
	// 	}
	// }
	// avg = sum / num;

	// ssum = 0;
	// for (int i = 0; i < w; i++)
	// {
	// 	for (int j = 0; j < h; j++)
	// 	{
	// 		if (opencvDepthMat.at<float>(y + j, x + i) > 0 && opencvDepthMat.at<float>(y + j, x + i) > (avg - 3 * standard) && opencvDepthMat.at<float>(y + j, x + i) < (avg + 3 * standard))
	// 		{
	// 			ssum += pow((opencvDepthMat.at<float>(y + j, x + i) - avg), 2);
	// 		}
	// 	}
	// }
	// standard = std::sqrt(ssum / (num - 1));

	// std::cout << a1 * 1.0 / num << std::endl;
	// std::cout << a2 * 1.0 / num << std::endl;
	// std::cout << a3 * 1.0 / num << std::endl;
	// std::cout << a4 * 1.0 / num << std::endl;
	// std::cout << avg << std::endl;
	// std::cout << standard << std::endl;

	// // int sum = 0;
	// // float num = 0;
	// // for (int i = 0; i < 200; i++) //5 result=6
	// // {
	// // 		for (int j = 0; j < 130; j++)
	// // 		{
	// // 			if (opencvDepthMat.at<float>(415 + j, 280 + i) > 0)
	// // 			{
	// // 				sum++;
	// // 				num += opencvDepthMat.at<float>(415 + j, 280 + i);
	// // 			}
	// // 		}
	// // }
	// // for (int i = 0; i < 80; i++) //10 result=14
	// // {
	// // 		for (int j = 0; j < 69; j++)
	// // 		{
	// // 			if (opencvDepthMat.at<float>(320 + j, 395 + i) > 0)
	// // 			{
	// // 				sum++;
	// // 				num += opencvDepthMat.at<float>(320 + j, 395 + i);
	// // 			}
	// // 		}
	// // }
	// // for (int i = 0; i < 80; i++) //15 result=25
	// // {
	// // 		for (int j = 0; j < 60; j++)
	// // 		{
	// // 			if (opencvDepthMat.at<float>(260 + j, 395 + i) > 0)
	// // 			{
	// // 				sum++;
	// // 				num += opencvDepthMat.at<float>(260 + j, 395 + i);
	// // 			}
	// // 		}
	// // }
	// // for (int i = 0; i < 60; i++) //20 result=48
	// // {
	// // 		for (int j = 0; j < 50; j++)
	// // 		{
	// // 			if (opencvDepthMat.at<float>(245 + j, 400 + i) > 0)
	// // 			{
	// // 				sum++;
	// // 				num += opencvDepthMat.at<float>(245 + j, 400 + i);
	// // 			}
	// // 		}
	// // }
	// // std::cout << num * 1.0 / sum << std::endl;

	// // //save the disparity image
	// // std::string left_img_stamp = to_string(left_msg->header.stamp.sec) + "_" + to_string(left_msg->header.stamp.nsec);
	// // std::string right_img_stamp = to_string(right_msg->header.stamp.sec) + "_" + to_string(right_msg->header.stamp.nsec);

	// // std::string base = basedSaveFilePath;
	// // std::string disparity = DispairtyPath;
	// // std::string retificed = RectifiedPath;

	// // cv::imwrite(base + retificed + "right/" + right_img_stamp + ".bmp", right_img);

	cv::Size saveImageSize;
	saveImageSize.width = II_IMAGE_WIDE;
	saveImageSize.height = II_IMAGE_HEIGHT;

	// double writeDisparityTime = (double)cvGetTickCount();
	// FILE *src_fp = NULL;
	// int src_ret = 0;
	// std::string path = "/home/wangsen/桌面/0.disparity"; //named by the right images
	// char *cpath = &path[0];
	// if ((src_fp = fopen(cpath, "wb")) != NULL)
	// {
	// 	src_ret = fwrite(opencvDisparityMat.data, saveImageSize.height * saveImageSize.width * sizeof(unsigned short), 1, src_fp);
	// 	fclose(src_fp);
	// }
	// writeDisparityTime = (double)cvGetTickCount() - writeDisparityTime;
	// std::cout << " write disparity time : " << writeDisparityTime / ((double)cvGetTickFrequency() * 1000) << std::endl;

	// cv::imshow("rectified_right", right_img);
	// cv::imshow("Disparity_Color", *pOpencvDisparityMatDisplay);
	// cv::waitKey(0);

	// if (pOpencvDisparityMatDisplay != NULL)
	// 	cv::setMouseCallback("Disparity_Color", my_mouse_callback, (void *)&opencvDepthMat);
	// cv::imwrite(base+disparity+"images/"+right_img_stamp+".bmp", *pOpencvDisparityMatDisplay);

	// char k = cv::waitKey(10);
}