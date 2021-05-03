#pragma once
#include "mytest/IiBasicDef.h"

class CIIStereoCalibrator
{
public:
	CIIStereoCalibrator(void);
	~CIIStereoCalibrator(void);

	//ymlFileURI: the xml file which has the calibrating parameters for left and right image rectifying.
	//expectImgSize: the expected image size after calibrating
	//zoomRatio: the zoom control value for the original left and right image to enhance the details
	void init(cv::String ymlFileURI, cv::Size expectImgSize, float wideRatio, float heightRatio);

	//left: the original left image
	//right: the original right image
	//rectified_left: the rectified left image
	//rectified_right: the rectified right image
	void rectifyImages(const cv::Mat &left, const cv::Mat &right, cv::Mat &rectifed_left, cv::Mat &rectified_right);

	void release();

private:
	double focus;
	double BF;
	double cx, cy;
	double baseline;
	cv::Size imgSize;
	cv::Rect overlap;
	cv::Mat L_map1, L_map2, R_map1, R_map2;
};
