#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;

// 点类
class point
{
public:
    double x, z, y;
    int ptsCnt, cluster;
    double getDis(const point &ot);
};

// dbsacn聚类
class DBSCAN
{
public:
    int n, minPts, size, clusterIdx;
    double eps;
    vector<point> points;
    vector<vector<int>> adjpoints;
    vector<vector<point>> cluster;

    DBSCAN(int n, double eps, int minPts);
    void run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
    void dfs(int now, int c);
    void checkNearpoints();
    bool isCoreObject(int idx);
    vector<vector<point>> get_cluster();
};

#endif