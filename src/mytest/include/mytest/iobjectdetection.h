
#ifndef __IOBJECTDETECTION_H
#define __IOBJECTDETECTION_H TRUE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace lidar
{
struct Detected_Obj
{
  jsk_recognition_msgs::BoundingBox bounding_box_;

  pcl::PointXYZ min_point_;
  pcl::PointXYZ max_point_;
  pcl::PointXYZ centroid_;

  int id;
  int semantic;
  float confidence;
  std::vector<cv::Point> cells;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointCloud<pcl::PointXYZI> polygon;
};
struct Quaternion
{
  double w, x, y, z;
};
class IObjectDetection
{
public:
  virtual bool Detect_Object(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pointcloud, boost::shared_ptr<std::vector<Detected_Obj>> obj_list) = 0;
};
} // namespace lidar

#endif