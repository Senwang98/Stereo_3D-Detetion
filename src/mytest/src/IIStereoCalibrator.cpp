#include "mytest/IIStereoCalibrator.h"
#include "mytest/IiBasicDef.h"
#include <stdlib.h>

CIIStereoCalibrator::CIIStereoCalibrator(void)
{
}

CIIStereoCalibrator::~CIIStereoCalibrator(void)
{
}

// �ؼ������ȷ�� new�Ĵ�С�� �����Ĵ�С
void CIIStereoCalibrator::init(cv::String ymlFileURI, cv::Size expectImgSize, float wideRatio, float heightRatio)
{
	cv::Mat cameraMatrix1 = cv::Mat::zeros(3, 3, CV_64FC1), distCoeffs1 = cv::Mat::zeros(5, 1, CV_64FC1);
	cv::Mat cameraMatrix2 = cv::Mat::zeros(3, 3, CV_64FC1), distCoeffs2 = cv::Mat::zeros(5, 1, CV_64FC1);
	cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1), T = cv::Mat::zeros(3, 1, CV_64FC1);

	cv::FileStorage fs;
	fs.open(ymlFileURI, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Cannot open YML files." << std::endl;
		exit(-1);
	}
	fs["cameraMatrix1"] >> cameraMatrix1;
	fs["distCoeffs1"] >> distCoeffs1;
	fs["cameraMatrix2"] >> cameraMatrix2;
	fs["distCoeffs2"] >> distCoeffs2;

	fs["R"] >> R;
	fs["T"] >> T;

	fs.release();

	expectImgSize.width -= (expectImgSize.width % 4);
	expectImgSize.height -= (expectImgSize.height % 4);

	cv::Size imageSize, newImageSize;
	imageSize.width = II_IMAGE_WIDE;
	imageSize.height = II_IMAGE_HEIGHT;

	newImageSize.width = wideRatio * II_IMAGE_WIDE;
	newImageSize.height = heightRatio * II_IMAGE_HEIGHT;
	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect roi1, roi2;

	cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 1, newImageSize, &roi1, &roi2);

	std::cout << "Roi1.x = " << roi1.x << " Roi1.y = " << roi1.y << std::endl;
	std::cout << "Roi2.x = " << roi2.x << " Roi2.y = " << roi2.y << std::endl;
	std::cout << "Roi1.width = " << roi1.width << " Roi1.height = " << roi1.height << std::endl;
	std::cout << "Roi2.width = " << roi2.width << " Roi2.height = " << roi2.height << std::endl;

	overlap.x = std::max(roi1.x, roi2.x);
	overlap.y = std::max(roi1.y, roi2.y);

	int x = roi1.x + roi1.width;
	overlap.width = std::min(roi1.width + roi1.x - 1, roi2.width + roi2.x - 1) - overlap.x + 1;
	overlap.height = std::min(roi1.height + roi1.y - 1, roi2.height + roi2.y - 1) - overlap.y + 1;
	overlap.width -= overlap.width % 4;
	overlap.height -= overlap.height % 4;

	int nExpectW, nExpectH;
	nExpectW = expectImgSize.width > overlap.width ? overlap.width : expectImgSize.width;
	nExpectH = expectImgSize.height > overlap.height ? overlap.height : expectImgSize.height;

	int margin_x = (overlap.width - nExpectW) / 2;
	int margin_y = (overlap.height - nExpectH) / 2;

	overlap.x += margin_x;
	overlap.y += margin_y;
	overlap.x -= overlap.x % 2;
	overlap.y -= overlap.y % 2;
	overlap.width = nExpectW;
	overlap.height = nExpectH;

	imgSize.width = overlap.width;
	imgSize.height = overlap.height;

	BF = fabs(P2.at<double>(0, 3));
	focus = P2.at<double>(0, 0);
	cx = P2.at<double>(0, 2) - overlap.x;
	cy = P2.at<double>(1, 2) - overlap.y;
	baseline = fabs(BF / focus);

	// for (int i = 0; i < P2.rows; i++)
	// {
	// 	for (int j = 0; j < P2.cols; j++)
	// 	{
	// 		std::cout << P2.at<double>(i, j) << " ";
	// 	}
	// 	std::cout << std::endl;
	// }

	//print();

	std::cout << cv::format("Principal Point: [%0.2f, %0.2f]", cx, cy) << std::endl;
	std::cout << cv::format("Focus: %0.6f", focus) << std::endl;
	std::cout << cv::format("Baseline: %0.2f", baseline) << std::endl;
	std::cout << cv::format("BF: %0.6f", BF) << std::endl;
	std::cout << cv::format("Rectified Image Size: [%d, %d]", imgSize.width, imgSize.height) << std::endl;

	initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, newImageSize, CV_16SC2, L_map1, L_map2);
	initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, newImageSize, CV_16SC2, R_map1, R_map2);
}

void CIIStereoCalibrator::rectifyImages(const cv::Mat &left, const cv::Mat &right, cv::Mat &rectifed_left, cv::Mat &rectified_right)
{

	cv::Mat L_rview, R_rview;

	remap(left, L_rview, L_map1, L_map2, cv::INTER_LINEAR);
	remap(right, R_rview, R_map1, R_map2, cv::INTER_LINEAR);

	L_rview(overlap).copyTo(rectifed_left);
	R_rview(overlap).copyTo(rectified_right);
}

void CIIStereoCalibrator::release()
{
}
