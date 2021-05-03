#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <iostream>
#include <cstring>
#include <string.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

#include <Eigen/Core>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pangolin/pangolin.h>

#include "mytest/IiBasicDef.h"
#include "mytest/colorCCA.h"

using namespace std;
using namespace message_filters;
using namespace sensor_msgs;

#ifdef __unix
#define fopen_s(pFile, filename, mode) ((*(pFile)) = fopen((filename), (mode))) == NULL
#endif

DEFINE_string(disparity_path, "", "path of disparity disparity_img");
DEFINE_string(rightimg_path, "", "path of right disparity_img");
DEFINE_int32(threshold_similar, 0, "threshold of Euclid distance of two pixels");
DEFINE_double(baseline, 0.0, "baseline of two cameras");
DEFINE_double(focal, 0.0, "focal length of camera");
DEFINE_double(resx, 0.1, "resolution in x");
DEFINE_double(resz, 0.1, "resolution in z");

std::string d_path = "";
std::string img_path = "";

void showDisparity(cv::Mat &disparityMat, cv::Mat &rightColorMat, cv::Mat &seg, int number)
{
    FILE *src_fp = NULL;
    int src_ret = 0;
    cv::Size parseImageSize;
    parseImageSize.width = II_IMAGE_WIDE;
    parseImageSize.height = II_IMAGE_HEIGHT;
    cv::resize(seg, seg, parseImageSize);

    // 读入右图
    // std::string parseFileNameURL;
    // parseFileNameURL = rightColorPath;
    // cv::Mat rightColorMat = cv::imread(parseFileNameURL, 1);

    // 语义分割图染色
    cv::Mat colorsem(seg.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < seg.rows; ++i)
    {
        uchar *curRow = seg.ptr<uchar>(i);
        for (int j = 0; j < seg.cols; ++j)
        {
            uchar *cur_data = curRow + j;
            colorsem.ptr<cv::Vec3b>(i)[j] = colorCCA::make_color((int)*cur_data);
        }
    }
    cv::imshow("semantic", colorsem);
    //cv::imwrite("/home/wangsen/桌面/bino_data/color_seg/" + std::to_string(number) + ".bmp", colorsem);

    // 读取视差图数据
    // cv::Mat disparityMat = cv::Mat::zeros(parseImageSize, CV_16UC1);
    // if (fopen_s(&src_fp, disparityFilePath, "rb") == 0)
    // {
    //     src_ret = fread(disparityMat.data, parseImageSize.height * parseImageSize.width * sizeof(unsigned short), 1, src_fp);
    //     fclose(src_fp);
    // }
    // cv::imshow("disp1", disparityMat);

    //// turn disparity map to pseudo-color ////
    cv::Mat disparity32F = cv::Mat::zeros(parseImageSize, CV_32FC1);
    float *pDisparity32FData = (float *)disparity32F.data;
    unsigned short *tempDisparity = (unsigned short *)disparityMat.data;
    for (int i = 0; i < disparity32F.cols * disparity32F.rows; i++)
    {
        // //        *pDisparity32FData++ = *tempDisparity++ / 256.0;   // EBM disparity
        *pDisparity32FData++ = *tempDisparity++ / 16.0; // opencv sgm disparity

        // // .bin -> disparity
        // float confident = *tempDisparity & 0x7;
        // if (confident > 3)
        // {
        //     *pDisparity32FData = (*tempDisparity >> 3) & 0x7FF;
        //     *pDisparity32FData = *pDisparity32FData / 16.f;
        // }
        // else
        // {
        //     *pDisparity32FData = 2048;
        // }
        // // std::cout << *tempDisparity << " ";
        // *pDisparity32FData++;
        // *tempDisparity++;
    }

    //  显示
    // cv::Mat tmp_disp32f;
    // disparity32F.convertTo(tmp_disp32f, CV_32F, 1.0 / 16);                                //除以16得到真实视差值
    // cv::Mat opencvDisparityMat8U = cv::Mat(tmp_disp32f.rows, tmp_disp32f.cols, CV_8UC1);  //显示
    // normalize(tmp_disp32f, opencvDisparityMat8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    // cv::imshow("Disparity_8U", opencvDisparityMat8U);

    cv::Mat *pDisparityMatDisplay = new cv::Mat(II_IMAGE_HEIGHT, II_IMAGE_WIDE, CV_8UC3);
    generatePseudoColorImageForDisparity((float *)disparity32F.data, II_IMAGE_WIDE, II_IMAGE_HEIGHT, pDisparityMatDisplay);
    cv::imshow("EBM_Disparity_Color", *pDisparityMatDisplay);
    //cv::imwrite("/home/wangsen/桌面/bino_data/EBM_Disparity_Color/" + std::to_string(number) + ".bmp", *pDisparityMatDisplay);

    // 拟合地面
    // 生成点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_test(new pcl::PointCloud<pcl::PointXYZ>);

    for (int v = 0; v < disparity32F.rows; v++)
    {
        for (int u = 0; u < disparity32F.cols; u++)
        {
            //Eigen::Vector4d point(0, 0, 0, disparity32F.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色
            // 根据双目模型计算 point 的位置
            float d = disparity32F.at<float>(v, u);
            // cout << d << endl;
            // if (d >= 58.2 || d <= 2.2)
            // {
            //     continue;
            // }
            if (d >= 51.26 || d <= 2.56) // pointgrey
            {
                continue;
            }
            pcl::PointXYZRGB point;
            point.z = (FX * II_Baseline) / d;
            point.y = (v - II_CENTER_V) * point.z / FY;
            point.x = (u - II_CENTER_U) * point.z / FX;
            point.b = rightColorMat.ptr<uchar>(v)[u * 3];
            point.g = rightColorMat.ptr<uchar>(v)[u * 3 + 1];
            point.r = rightColorMat.ptr<uchar>(v)[u * 3 + 2];
            pointcloud->points.push_back(point);
            pcl::PointXYZ point_xyz;
            point_xyz.z = (FX * II_Baseline) / d;
            point_xyz.y = (v - II_CENTER_V) * point_xyz.z / FY;
            point_xyz.x = (u - II_CENTER_U) * point_xyz.z / FX;
            pointcloud_test->points.push_back(point_xyz);
        }
    }
    cout << "点云共有 : " << pointcloud->points.size() << "个点" << endl;
    //创建一个模型参数对象，用于记录结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建一个分割器
    pcl::SACSegmentation<pcl::PointXYZ> pcl_seg;
    // Optional
    pcl_seg.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状
    pcl_seg.setModelType(pcl::SACMODEL_PLANE);
    //分割方法：随机采样法
    pcl_seg.setMethodType(pcl::SAC_RANSAC);
    //设置误差容忍范围
    pcl_seg.setDistanceThreshold(0.1);
    //输入点云
    pcl_seg.setInputCloud(pointcloud_test);
    //分割点云
    pcl_seg.segment(*inliers, *coefficients);

    // Plane Model: ax+by+cz+d=0; saved in *coefficients
    float a, b, c, d;
    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];

    int i, j, k;
    int size = pointcloud->size();
    //------------------------ Projection ------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ ground_point;
    pcl::PointXYZRGB object_point;
    int remove_ground_cnt = 0;
    for (i = 0; i < size; i++)
    {
        float x, y, z;
        x = pointcloud->at(i).x;
        y = pointcloud->at(i).y;
        z = pointcloud->at(i).z;
        ground_point.x = ((b * b + c * c) * x - a * (b * y + c * z + d)) / (a * a + b * b + c * c);
        ground_point.y = ((a * a + c * c) * y - b * (a * x + c * z + d)) / (a * a + b * b + c * c);
        ground_point.z = ((b * b + a * a) * z - c * (a * x + b * y + d)) / (a * a + b * b + c * c);

        ground->push_back(ground_point);
        if (y < ground_point.y - 50) // && y > ground_point.y - 2500
        {
            object_point.x = x;
            object_point.y = y;
            object_point.z = z;
            object_point.r = pointcloud->at(i).r;
            object_point.g = pointcloud->at(i).g;
            object_point.b = pointcloud->at(i).b;
            object->push_back(object_point);
        }
        else
        {
            float d = (FX * II_Baseline) / z;
            int v = int(II_CENTER_V + (y * FY) / z);
            int u = int(II_CENTER_U + (x * FX) / z);
            disparity32F.at<float>(v, u) = 0;
            remove_ground_cnt++;
        }
    }
    cout << "去除地面过程中删除的点数为:" << remove_ground_cnt << endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer);
    view->addPointCloud(object);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(ground, 0, 128, 0);
    view->addPointCloud(ground, color_handler, "show_plane");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "show_plane");
    while (!view->wasStopped())
        view->spinOnce();

    // 视差、分割融合
    int seg_dis_remove = 0;
    for (int i = 0; i < seg.rows; ++i)
    {
        uchar *curRow = seg.ptr<uchar>(i);
        for (int j = 0; j < seg.cols; ++j)
        {
            uchar *cur_data = curRow + j;
            // if ((int)*cur_data != 0 || (disparity32F.at<float>(i, j) >= 52.2) || (disparity32F.at<float>(i, j) <= 3.26))
            // {
            //     disparity32F.at<float>(i, j) = 0;
            // }
            // if ((int)*cur_data != 0 || (disparity32F.at<float>(i, j) >= 58.2) || (disparity32F.at<float>(i, j) <= 2.2))
            // {
            //     disparity32F.at<float>(i, j) = 0;
            //     seg_dis_remove++;
            // }
            if (!((int)*cur_data == 0 || (int)*cur_data == 16) || (disparity32F.at<float>(i, j) >= 51.26) || (disparity32F.at<float>(i, j) <= 2.56))
            {
                disparity32F.at<float>(i, j) = 0;
                seg_dis_remove++;
            }
        }
    }
    cout << "通过视差、分割融合去除的点的个数为：" << seg_dis_remove << endl;
    // cv::imwrite("/home/wangsen/桌面/bino_data/disp_seg/" + std::to_string(number) + ".bmp", disparity32F);

    // 图片预处理
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));
    cv::morphologyEx(disparity32F, disparity32F, cv::MORPH_CLOSE, element);
    cv::medianBlur(disparity32F, disparity32F, 5);

    // CCA
    cv::Mat pLabel = disparity32F.clone();
    colorCCA cca = colorCCA(disparity32F, pLabel, II_Baseline, II_Focus, FLAGS_resx, FLAGS_resz, II_CENTER_U, II_CENTER_V, rightColorMat);
    cca.colorsegment_seedfill(0);
    cca.getBbox(number); // 得到3D以及2D边框

    // 展示CCA聚类结果
    cv::Mat pLabel_show = cv::Mat::zeros(disparity32F.size(), CV_8UC3);
    for (int i = 0; i < pLabel_show.rows; ++i)
    {
        for (int j = 0; j < pLabel_show.cols; ++j)
        {
            if (cca.label_count[(int)pLabel.at<float>(i, j)] != 0)
            {
                pLabel_show.at<cv::Vec3b>(i, j) = colorCCA::make_color((int)pLabel.at<float>(i, j));
            }
        }
    }
    cv::imshow("pLabel", pLabel_show);
    //cv::imwrite("/home/wangsen/桌面/bino_data/pLabel/" + std::to_string(number) + ".bmp", pLabel_show);

    for (int i = 0; i < cca.BBox2D.size(); ++i)
    {
        // 远
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontdownright[0], cca.BBox2D[i].frontdownright[1]),
                 cv::Point(cca.BBox2D[i].frontdownleft[0], cca.BBox2D[i].frontdownleft[1]),
                 cv::Scalar(255, 0, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontupright[0], cca.BBox2D[i].frontupright[1]),
                 cv::Point(cca.BBox2D[i].frontupleft[0], cca.BBox2D[i].frontupleft[1]),
                 cv::Scalar(255, 0, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontdownright[0], cca.BBox2D[i].frontdownright[1]),
                 cv::Point(cca.BBox2D[i].frontupright[0], cca.BBox2D[i].frontupright[1]),
                 cv::Scalar(255, 0, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontdownleft[0], cca.BBox2D[i].frontdownleft[1]),
                 cv::Point(cca.BBox2D[i].frontupleft[0], cca.BBox2D[i].frontupleft[1]),
                 cv::Scalar(255, 0, 0), 1);
        // // 近
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].backdownright[0], cca.BBox2D[i].backdownright[1]),
                 cv::Point(cca.BBox2D[i].backdownleft[0], cca.BBox2D[i].backdownleft[1]),
                 cv::Scalar(0, 255, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].backupright[0], cca.BBox2D[i].backupright[1]),
                 cv::Point(cca.BBox2D[i].backupleft[0], cca.BBox2D[i].backupleft[1]),
                 cv::Scalar(0, 255, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].backdownright[0], cca.BBox2D[i].backdownright[1]),
                 cv::Point(cca.BBox2D[i].backupright[0], cca.BBox2D[i].backupright[1]),
                 cv::Scalar(0, 255, 0), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].backdownleft[0], cca.BBox2D[i].backdownleft[1]),
                 cv::Point(cca.BBox2D[i].backupleft[0], cca.BBox2D[i].backupleft[1]),
                 cv::Scalar(0, 255, 0), 1);
        // 四条深度线
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontdownright[0], cca.BBox2D[i].frontdownright[1]),
                 cv::Point(cca.BBox2D[i].backdownright[0], cca.BBox2D[i].backdownright[1]),
                 cv::Scalar(0, 0, 255), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontupright[0], cca.BBox2D[i].frontupright[1]),
                 cv::Point(cca.BBox2D[i].backupright[0], cca.BBox2D[i].backupright[1]),
                 cv::Scalar(0, 0, 255), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontdownleft[0], cca.BBox2D[i].frontdownleft[1]),
                 cv::Point(cca.BBox2D[i].backdownleft[0], cca.BBox2D[i].backdownleft[1]),
                 cv::Scalar(0, 0, 255), 1);
        cv::line(rightColorMat,
                 cv::Point(cca.BBox2D[i].frontupleft[0], cca.BBox2D[i].frontupleft[1]),
                 cv::Point(cca.BBox2D[i].backupleft[0], cca.BBox2D[i].backupleft[1]),
                 cv::Scalar(0, 0, 255), 1);
    }
    cv::imshow("Right_Color", rightColorMat);
    // cv::imwrite("/home/wangsen/桌面/zed_result/5_degree0" + std::to_string(number) + ".bmp", rightColorMat);

    cv::waitKey(1000);
}

// void mycallback(const sensor_msgs::ImageConstPtr &msg)
// {
//     cv::Mat disp = cv_bridge::toCvShare(msg, "mono16")->image;
//     int i=0;
//     std::string right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_0degree/40/right/" + std::to_string(i) + ".bmp";
//     cv::Mat seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/40/degree0/" + std::to_string(i) + ".bmp", 0);
//     showDisparity(disp, right_path.c_str(), seg, i);
//     // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//     // cv::waitKey(0);
// }

class solve
{
public:
    solve(ros::NodeHandle &_nh)
    {
        nh = _nh;
        message_filters::Subscriber<Image> sub_disp(nh, "/camera/disparity", 100);
        message_filters::Subscriber<Image> sub_seg(nh, "/camera/segment", 100);
        message_filters::Subscriber<Image> sub_right(nh, "/camera/rectified_right", 100);
        typedef sync_policies::ApproximateTime<Image, Image, Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_disp, sub_seg, sub_right);
        sync.registerCallback(boost::bind(&solve::mycallback, this, _1, _2, _3));
        ros::spin();
    }
    ~solve() {}
    void mycallback(const sensor_msgs::ImageConstPtr &disp_msg, const sensor_msgs::ImageConstPtr &seg_msg, const sensor_msgs::ImageConstPtr &right_msg)
    {
        cv::Mat disp = cv_bridge::toCvShare(disp_msg, "mono16")->image;
        cv::Mat seg = cv_bridge::toCvShare(seg_msg, "mono8")->image;
        cv::Mat right = cv_bridge::toCvShare(right_msg, "bgr8")->image;
        // cv::imshow("main right",right);
        // cv::waitKey(0);
        // int i = disp_msg->header.seq % 3;
        // std::string right_path = "/home/wangsen/test_image/right/" + std::to_string(i) + ".bmp";
        int i = 0;
        // std::string right_path = "/home/wangsen/data0420/2/10/rect_right/0.bmp";
        // cv::Mat img = cv::imread(right_path, 1);
        std::cout << "main.cpp accept three pic ok!" << std::endl;
        showDisparity(disp, right, seg, i);
    }

private:
    ros::NodeHandle nh;
    message_filters::Subscriber<Image> *sub_disp;
    message_filters::Subscriber<Image> *sub_seg;
    message_filters::Subscriber<Image> *sub_right;
};

// void mycallback(const sensor_msgs::ImageConstPtr &disp_msg, const sensor_msgs::ImageConstPtr &seg_msg)
// {
//     cv::Mat disp = cv_bridge::toCvShare(disp_msg, "mono16")->image;
//     cv::Mat seg = cv_bridge::toCvShare(seg_msg, "mono8")->image;
//     int i = 0;
//     std::string right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_0degree/40/right/" + std::to_string(i) + ".bmp";
//     showDisparity(disp, right_path.c_str(), seg, i);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    // int dis = atoi(argv[1]);
    // int degree = atoi(argv[2]);
    // int st = atoi(argv[3]);
    // int end = atoi(argv[4]);
    ros::NodeHandle n;
    // image_transport::ImageTransport it(n);
    // image_transport::Subscriber sub = it.subscribe("/camera/disparity", 1, mycallback);
    solve t(n);
    // message_filters::Subscriber<Image> sub_disp(n, "/camera/disparity", 1);
    // message_filters::Subscriber<Image> sub_seg(n, "/camera/segment", 1);
    // typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_disp, sub_seg);
    // sync.registerCallback(boost::bind(&mycallback, _1, _2));
    ros::spin();
    /*
    // Apollo 测试
    jsk_recognition_msgs::BoundingBox bbx;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point1(-10, 10, 0);
    pointcloud->push_back(point1);
    pcl::PointXYZ point2(-10, 30, 0);
    pointcloud->push_back(point2);
    pcl::PointXYZ point3(10, 10, 0);
    pointcloud->push_back(point3);
    pcl::PointXYZ point4(10, 30, 0);
    pointcloud->push_back(point4);

    pcl::PointXYZ point5(-10, 10, 10);
    pointcloud->push_back(point5);
    pcl::PointXYZ point6(-10, 30, 10);
    pointcloud->push_back(point6);
    pcl::PointXYZ point7(10, 10, 10);
    pointcloud->push_back(point7);
    pcl::PointXYZ point8(10, 30, 10);
    pointcloud->push_back(point8);

    pcl::PointXYZ point9(0, 35, 0);
    pointcloud->push_back(point9);
    pcl::PointXYZ point10(0, 35, 10);5
    pointcloud->push_back(point10);

    // pcl::PointXYZ point11(0, 1, 0);
    // pointcloud->push_back(point11);
    // pcl::PointXYZ point12(0, 1, 10);
    // pointcloud->push_back(point12);

    Build_Object(pointcloud,bbx);
    cout<<"bounding_box.pose.position.x = "<<bbx.pose.position.x<<endl;
    cout<<"bounding_box.pose.position.y = "<<bbx.pose.position.y<<endl;
    cout<<"bounding_box.pose.position.z = "<<bbx.pose.position.z<<endl;

    cout<<"bbx.dimensions.x = "<<bbx.dimensions.x<<endl;
    cout<<"bbx.dimensions.y = "<<bbx.dimensions.y<<endl;
    cout<<"bbx.dimensions.z = "<<bbx.dimensions.z<<endl;
    */

    // ros::Rate loop_rate(10);
    // google::ParseCommandLineFlags(&argc, &argv, true);
    // std::string disparity_path, right_path;
    // cv::Mat seg;
    // for (int i = st; i <= end; i++)
    // {
    //     if (dis == 0 && degree == 0)
    //     {
    //         disparity_path = "/home/wangsen/桌面/bino_data/disp/disp_s16_0000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/bino_data/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/bino_data/seg/" + std::to_string(i - 1) + ".bmp", 0);
    //     }

    //     if (dis == 5 && degree == 0) // 5米　0°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_0degree/5/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_0degree/5/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/5/degree0/" + std::to_string(i) + ".bmp", 0);
    //     }
    //     if (dis == 5 && degree == 90) // 5米　90°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_90degree/5/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_90degree/5/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/5/degree90/" + std::to_string(i) + ".bmp", 0);
    //     }

    //     if (dis == 10 && degree == 0) // 10米　0°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.9/disp/10/degree0/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.9/source/10/degree0/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.9/seg/10/degree0/" + std::to_string(i) + ".bmp", 0);
    //     }
    //     if (dis == 10 && degree == 90) // 10米　90°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.9/disp/10/degree90/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.9/source/10/degree90/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.9/seg/10/degree90/" + std::to_string(i) + ".bmp", 0);
    //     }

    //     if (dis == 20 && degree == 0) // 20米　0°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_0degree/20/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_0degree/20/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/20/degree0/" + std::to_string(i) + ".bmp", 0);
    //     }

    //     if (dis == 20 && degree == 90) // 20米　90°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_90degree/20/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_90degree/20/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/20/degree90/" + std::to_string(i) + ".bmp", 0);
    //     }

    //     if (dis == 40 && degree == 0) // 40米　0°
    //     {
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_0degree/40/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_0degree/40/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/40/degree0/" + std::to_string(i) + ".bmp", 0);
    //         FILE *src_fp = NULL;
    //         int src_ret = 0;
    //         cv::Size parseImageSize;
    //         parseImageSize.width = II_IMAGE_WIDE;
    //         parseImageSize.height = II_IMAGE_HEIGHT;
    //         cv::Mat disparityMat = cv::Mat::zeros(parseImageSize, CV_16UC1);
    //         const char * disparityFilePath = disparity_path.c_str();
    //         if (fopen_s(&src_fp, disparityFilePath, "rb") == 0)
    //         {
    //             src_ret = fread(disparityMat.data, parseImageSize.height * parseImageSize.width * sizeof(unsigned short), 1, src_fp);
    //             fclose(src_fp);
    //         }
    //         showDisparity(disparityMat, right_path.c_str(), seg, i);
    //     }

    //     if (dis == 40 && degree == 90) // 40米　90°
    //     {
    //         if (i == 0) //　此处的语义分割图跟五米的某张分割图一样
    //             continue;
    //         disparity_path = "/home/wangsen/桌面/toWangsen4.18/disp/white_90degree/40/disp/disp_s16_000000000" + std::to_string(i) + ".bin";
    //         right_path = "/home/wangsen/桌面/toWangsen4.18/source/white_90degree/40/right/" + std::to_string(i) + ".bmp";
    //         seg = cv::imread("/home/wangsen/桌面/toWangsen4.18/seg/40/degree90/" + std::to_string(i) + ".bmp", 0);
    //     }
    //     // showDisparity(disparity_path.c_str(), right_path.c_str(), seg, i);
    //     cout << "Pic " << i << "finish!" << endl;
    //     cout << endl;
    // }
    return 0;
}