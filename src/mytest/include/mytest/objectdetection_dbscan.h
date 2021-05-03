#ifndef __OBJECTDETECTION_DBSCAN_H
#define __OBJECTDETECTION_DBSCAN_H TRUE

#include "iobjectdetection.h"
#include <queue>

#define KERNELXSIZE 3
#define KERNELYSIZE 3
namespace lidar
{
    class ObjectDetection_Dbscan : public IObjectDetection
    {
    public:
        ObjectDetection_Dbscan();
        ~ObjectDetection_Dbscan();

    public:
        virtual bool Detect_Object(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pointcloud, boost::shared_ptr<std::vector<Detected_Obj>> obj_list);

        /// \brief get cluster from an object's point cloud
        /// \param pointcloud
        /// \param tolerance  size of grid -- whose unit must be same as point cloud (m / mm)
        /// \param radius    radius of whole point cloud -- whose unit must be same as point cloud (m / mm)
        /// \param obj_list
        void Cluster_Object(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::vector<double> minXY, double tolerance, double radius, boost::shared_ptr<std::vector<Detected_Obj>> obj_list);
    };
} // namespace lidar

#endif