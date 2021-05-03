#include "mytest/IIOpencvDisparityCalculator.h"

CIIOpencvDisparityCalculator::CIIOpencvDisparityCalculator(void)
{
	m_pDisparityDataMat = NULL;
}

CIIOpencvDisparityCalculator::~CIIOpencvDisparityCalculator(void)
{
}

void CIIOpencvDisparityCalculator::init()
{
	m_ndisparities = 64; /**< Range of disparity */
	int SADWindowSize = 15;	 /**< Size of the block window. Must be odd */

	disparityMat = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_16S);

	sgbm = cv::StereoSGBM::create(0, m_ndisparities, 7);
	sgbm->setPreFilterCap(63);
	sgbm->setBlockSize(11);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(m_ndisparities); //%16==0
	int cn = 1;
	sgbm->setP1(8 * cn * sgbm->getBlockSize() * sgbm->getBlockSize());
	sgbm->setP2(32 * cn * sgbm->getBlockSize() * sgbm->getBlockSize());
	sgbm->setUniquenessRatio(10);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setMode(cv::StereoSGBM::MODE_HH);
	// m_ndisparities = ((II_IMAGE_WIDE_Rectified / 8) + 15) & -16; /**< Range of disparity */
	// int SADWindowSize = 15;	 /**< Size of the block window. Must be odd */

	// disparityMat = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_16S);

	// sgbm = cv::StereoSGBM::create(0, m_ndisparities, 3);
	// sgbm->setPreFilterCap(63);
	// sgbm->setBlockSize(11);
	// sgbm->setMinDisparity(0);
	// sgbm->setNumDisparities(m_ndisparities); //%16==0
	// int cn = 1;
	// sgbm->setP1(8 * cn * sgbm->getBlockSize() * sgbm->getBlockSize());
	// sgbm->setP2(32 * cn * sgbm->getBlockSize() * sgbm->getBlockSize());
	// sgbm->setUniquenessRatio(10);
	// sgbm->setDisp12MaxDiff(1);
	// sgbm->setSpeckleWindowSize(100);
	// sgbm->setSpeckleRange(32);
	// sgbm->setMode(cv::StereoSGBM::MODE_HH);

	m_disparityMatDisplay = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_8UC3);

	if (m_pDisparityDataMat == NULL)
		m_pDisparityDataMat = new cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_16UC1);
}

void CIIOpencvDisparityCalculator::calculateDisparity(const cv::Mat refImage, const cv::Mat targetImage, cv::Mat &disparityImage, cv::Mat &depthImage)
{

	cv::Mat leftGrey, rightGrey;
	cv::cvtColor(refImage, leftGrey, CV_BGR2GRAY);
	cv::cvtColor(targetImage, rightGrey, CV_BGR2GRAY);

	cv::Mat disparityMatDisplay = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_8UC3);

	sgbm->compute(leftGrey, rightGrey, disparityMat); //disparityMat 16λ��ֵÿ������4��С��λ
	//make the disparity align with right image
	unsigned short *disparityData = (unsigned short *)disparityMat.data;

	cv::Mat rightRefDisp = cv::Mat::zeros(refImage.rows, refImage.cols, CV_16UC1);
	unsigned short *pRefDispData = (unsigned short *)rightRefDisp.data;

	for (int j = 0; j < refImage.rows; j++)
	{
		for (int i = 0; i < refImage.cols; i++)
		{
			uchar refDisp = (*disparityData >> 4) & 0x00FF;
			int refDispIndex = i - refDisp;
			if (refDispIndex >= 0)
			{
				pRefDispData[j * refImage.cols + refDispIndex] = *disparityData << 4;
			}

			disparityData++;
		}
	}

	disparityMat.copyTo(disparityImage);
	//����õ����ͼ

	cv::Mat depth32F = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_32FC1);

	int length = refImage.cols * refImage.rows;

	float *pTempDepth = (float *)depth32F.data;
	unsigned short *pTempDisparity = (unsigned short *)disparityMat.data;

	for (int i = 0; i < length; i++)
	{
		//the disparity value
		float disparity = *pTempDisparity / 16.0;

		if (disparity == 0)
			*pTempDepth = 0;
		else
			*pTempDepth = II_BF / disparity;

		pTempDepth++;
		pTempDisparity++;
	} //new camera disparity
	depth32F.copyTo(depthImage);
}

cv::Mat *CIIOpencvDisparityCalculator::showColorDisparity()
{

	cv::Mat disparity32F = cv::Mat(II_IMAGE_HEIGHT_Rectified, II_IMAGE_WIDE_Rectified, CV_32FC1);

	float *pDisparity32FData = (float *)disparity32F.data;
	unsigned short *tempDisparity = (unsigned short *)disparityMat.data;

	for (int i = 0; i < disparity32F.cols * disparity32F.rows; i++)
	{
		*pDisparity32FData++ = *tempDisparity++ / 16.0;
	}
	// cv::Mat result;
	// cv::bilateralFilter(disparity32F, result, 3, 500, 15);

	generatePseudoColorImageForDisparity((float *)disparity32F.data, II_IMAGE_WIDE_Rectified, II_IMAGE_HEIGHT_Rectified, &m_disparityMatDisplay);

	return &m_disparityMatDisplay;
}

void CIIOpencvDisparityCalculator::destroy()
{

	disparityMat.release();
}
