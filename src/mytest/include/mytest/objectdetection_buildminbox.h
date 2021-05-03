#ifndef __OBJECTDETECTION_BUILDMINBOX_H
#define __OBJECTDETECTION_BUILDMINBOX_H   TRUE

#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE


#define EPSILON 1e-6
#define MAXX 100.0
#define MINX 0.0
#define MAXY 10.0
#define MINY -10.0
#define CELLSIZEX 0.2
#define CELLSIZEY 0.2
#define MINPOINTS 1
#define WIDTH (MAXY-MINY)/CELLSIZEY

struct Quaternion
{
    double w, x, y, z;
};

struct grid
{
    float x,y,minz,maxz;
    int cluster;
    pcl::PointCloud<pcl::PointXYZ> points;
};

bool Build_Object(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, jsk_recognition_msgs::BoundingBox & bounding_box);

void ComputeGeometricFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,jsk_recognition_msgs::BoundingBox & bounding_box);
//void PointCloud2Grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::vector<grid>> & grids);
void ComputePolygon2dxy(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, float & obj_height);
void GetCloudMinMax3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f* min_point, Eigen::Vector4f* max_point);
void ReconstructPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, jsk_recognition_msgs::BoundingBox & bounding_box, float & obj_height);
double ComputeAreaAlongOneEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, size_t first_in_point, Eigen::Vector3d* center,double* lenth, double* width, Eigen::Vector3d* dir);
Quaternion ToQuaternion(double yaw, double pitch, double roll);
void Quaternion2Euler(const Quaternion &q, double &yaw, double &pitch, double &roll);


#endif