#pragma once

#include "IiBasicDef.h"


class CIIOpencvDisparityCalculator
{
public:
	CIIOpencvDisparityCalculator(void);
	~CIIOpencvDisparityCalculator(void);

	void init();
	void calculateDisparity(const cv::Mat refImage, const cv::Mat targetImage, cv::Mat & disparityImage, cv::Mat& depthImage);
	void destroy();
	cv::Mat* showColorDisparity();


	cv::Mat disparityMat;  //orinal dispairty by opencv
	cv::Ptr<cv::StereoSGBM> sgbm;

	int m_ndisparities;
	cv::Mat* m_pDisparityDataMat;  //16U format
	cv::Mat m_disparityMatDisplay;

};

