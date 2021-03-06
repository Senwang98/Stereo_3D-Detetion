#pragma once

#include <opencv2/opencv.hpp>




#define generateAndDisplayPseudocolor4DisparitiesMacro_
static char* basedSaveFilePath = "D:/StereoRawData/20180126";  //???????????????洢?????·??????????????????н????????????????·??


#define ParseAllImagesInPredefinedDirAndGenerating_  //??????????????????????????????????????????
                                                     //1????????ж? ?????????洢?????·????????????д?????????
                                                     //   ????У???????????????????????????????У???????
                                                     //2????????????????????洢?????·?????ж????
                                                     //3??????????????????????????????????
                                                     //4???????????????????
                                                     //5????????????Opencv???????????????????????洢?????????????????


#define CHECK_BY_OLD_1 //???????????????????????????matlab??????????Ч??

//readFromCamera??????????zed????????????洢?????
//                          ???????????????????????????????????????????????????????????????????????????readDisparityFromFileDirectly
#define readFromCamera
//readDisparityFromFileDirectly?????????????????????????????????????????????
//                                         ????????????????????????????????????????洢????????????????????
#define readDisparityFromFileDirectly_1

//?????????????????洢·??????????
//      ????·??/????/????/????????
//      ???л???·????basedSaveFilePath
//      ????????????????????????????????????????
//              ????????????????????????????????ε?????????????????selectedTimeStampPath
//              ?????????????????1????????


#define ROIAreaSelectsByTwoPointsAndShowResultsInFiles_  //??У????????????????????????????????????????Ч??????????
                                                        //???????????????????????????????????????????????????ebm???????????л???opencv???
                                                        //?????????????????????????????
                                                        //??????????????????и?????????????????????????q??
                                                        //?????????е??????????????txt????У?????????У???????????????ebm????????opencv??sgbm??

static char* selectedTimeStampPath = "115841984"; //94929015
static char* OrignalPath = "orig";
static char* DispairtyPath = "disparity";
static char* OpencvDispairtyPath = "opencv_disparity";
static char* RGBPath = "rectified_right";

static char* DisparityPseudoColorPath = "disparityPseudoColor";
static char* OpencvDispairtyPseudoColorPath = "opencvDisparityPseudoColor";


#ifdef CHECK_BY_OLD_
	#define II_CENTER_U 693.73     //???????????2015??8????????
	#define II_CENTER_V 377.70  
	#define II_BF 91177.05       
	#define II_Focus 757.496  
	#define II_Baseline 120.37 

#else
	//#define II_CENTER_U 658.28    //????2017??9??13???????????20170914?????, enlarge co is 1.5
	//#define II_CENTER_V 401.86  
	//#define II_BF 96943.17       
	//#define II_Focus 805.1267  
	//#define II_Baseline 120.41 
  // for each new camera, it needs new values(for different enlarge coefficients, the above values should be different

  //#define II_CENTER_U 656.80    //????2017??9??13???????????20170914?????, enlarge co is 1.4
  //#define II_CENTER_V 398.14  
  //#define II_BF 90480.29178       
  //#define II_Focus 751.451544
  //#define II_Baseline 120.41 

// #define II_CENTER_U 654.85    //????2018??01??26???????????20180131?????, enlarge co is 1.4
// #define II_CENTER_V 336.91  
// #define II_BF 88434.714525       
// #define II_Focus 740.758106
// #define II_Baseline 119.38 

// #define II_CENTER_U 639.75    //????2018??01??26???????????20180201?????, enlarge co is 1.4
// #define II_CENTER_V 333.58  
// #define II_BF 87262.49309       
// #define II_Focus 731.035446
// #define II_Baseline 119.37 

// ZED
// #define II_CENTER_U 666.52    //基于2020年03月31日采集的数据，20180331完成的标定, 2020.4.22 change enlarge co to 1.5
// #define II_CENTER_V 333.27  
// #define II_BF 87321.552568       
// #define II_Focus 729.715566
// #define II_Baseline 119.67
// #define FX 729.715566
// #define FY 729.715566

// #define II_CENTER_U 652.379905    
// #define II_CENTER_V 351.596573 
// #define II_BF 87321.552568       
// #define II_Focus 729.715566
// #define II_Baseline 119.67
// #define FX 696.329408
// #define FY 697.198805

// PointGrey
// #define II_CENTER_U 527.70    //2020/03/06, left downsize to 1024*768, right crop to 1024*768, enlarge co is 1.4
// #define II_CENTER_V 403.48  
// #define II_BF 130468.980280     
// #define II_Focus 1631.928883
// #define II_Baseline 79.95

#define II_CENTER_U 523.37    //2020/04/23, left downsize to 1024*768, right crop to 1024*768, enlarge co is 1.1
#define II_CENTER_V 401.69 
#define II_BF 102517.844937  
#define II_Focus 1282.311181
#define II_Baseline 79.95
#define FX II_Focus
#define FY II_Focus

#endif

// #ifdef _ZED_CAM_

//typedef unsigned __int64 ULONGLONG;

#define II_IMAGE_HEIGHT  768
#define II_IMAGE_WIDE  1024
#define II_IMAGE_HEIGHT_Rectified  768
#define II_IMAGE_WIDE_Rectified  1024

// #define II_IMAGE_HEIGHT  720
// #define II_IMAGE_WIDE  1280
// #define II_IMAGE_HEIGHT_Rectified  720
// #define II_IMAGE_WIDE_Rectified  1280

typedef enum _IMAGE_MODE {
	LEFT_DEPTH, /*left disparity_img + depth disparity_img*/
	RAW_LEFT_RIGHT /* raw disparity_img */
}IMAGE_MODE;

// #endif

//need to modified it for each camera

#ifdef _WIN32  
#include <direct.h>  
#include <io.h>  
#elif _LINUX  
#include <stdarg.h>  
#include <sys/stat.h>  
#endif  

#ifdef _WIN32  
#define ACCESS _access  
#define MKDIR(a) _mkdir((a))  
#elif _LINUX  
#define ACCESS access  
#define MKDIR(a) mkdir((a),0755)  
#endif  


void ST_setCLUT_sub();

void generatePseudoColorImageForDisparity(float* pDisparityMap, int width, int height, cv::Mat* pColorImage);
