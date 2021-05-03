#include "mytest/objectdetection_dbscan.h"
#include "mytest/objectdetection_buildminbox.h"
#include <pcl_conversions/pcl_conversions.h>

namespace lidar
{
//    0 => 0-60m d=0.5
//    1 => >60m  d=0.5
const int block_num = 2;
const float block_radius[2] = {60, 120};
const float invalid_block_radius = 120;
const float cluster_tolerance[2] = {0.5, 1.0};
const int min_cluster_size = 50;
const int max_cluster_size = 5000;
const int min_points = 23; // minimum points number of a grid

ObjectDetection_Dbscan::ObjectDetection_Dbscan() {}

ObjectDetection_Dbscan::~ObjectDetection_Dbscan() {}

bool ObjectDetection_Dbscan::Detect_Object(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_pointcloud, boost::shared_ptr<std::vector<Detected_Obj>> obj_list)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pointcloud_blocks(block_num);

    for (size_t i = 0; i < pointcloud_blocks.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_block(new pcl::PointCloud<pcl::PointXYZI>);
        pc_block->header = in_pointcloud->header;
        pointcloud_blocks[i] = pc_block;
    }
    for (auto &current_point : in_pointcloud->points)
    {
        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));
        // if (origin_distance >= invalid_block_radius)
        // {
        //     continue;
        // }
        if (origin_distance < block_radius[0])
        {
            pointcloud_blocks[0]->points.push_back(current_point);
        }
        else if (origin_distance < block_radius[1])
        {
            pointcloud_blocks[1]->points.push_back(current_point);
        }
        else
        {
        }
    }
    for (int i = 0; i < pointcloud_blocks.size(); i++)
    {
        //        Cluster_Object(pointcloud_blocks[i], cluster_tolerance[i], block_radius[i], obj_list);
    }
    return true;
}

void ObjectDetection_Dbscan::Cluster_Object(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, std::vector<double> minXY, double tolerance, double radius, boost::shared_ptr<std::vector<Detected_Obj>> obj_list)
{
    // 首先计算筛子尺寸
    int grid_size = std::max((int)(2 * radius / tolerance), 1);
    if (grid_size == 1)
    {
        std::cout << "grid size wrong!" << endl;
        return;
    }
    ros::Time begin_time = ros::Time::now();
    int cloud_size = 0;
    //create 2d pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pointcloud, *cloud_2d);
    
    //　初始化筛子
    std::vector<std::vector<grid>> grids;
    grids.resize(grid_size);
    for (int i = 0; i < grids.size(); i++)
    {
        grids[i].resize(grid_size);
    }
    //　将点云投影到筛子中去
    for (int i = 0; i < cloud_2d->points.size(); i++)
    {
        int row = (int)((cloud_2d->points[i].y - minXY[1]) / tolerance);
        if (row >= grid_size)
        {
            row = grid_size - 1;
        }
        int col = (int)((cloud_2d->points[i].x - minXY[0]) / tolerance);
        if (col >= grid_size)
        {
            col = grid_size - 1;
        }
        grids[row][col].points.push_back(cloud_2d->points[i]);
        grids[row][col].minz = std::numeric_limits<float>::max();
        grids[row][col].maxz = std::numeric_limits<float>::min();
    }
    //　如果一个网格中点云个数少于设定值，cluster设置为-1
    for (int row = 0; row < grids.size(); row++)
    {
        for (int col = 0; col < grids[row].size(); col++)
        {
            if (grids[row][col].points.size() > cloud_2d->points.size() || grids[row][col].points.size() <= 0)
            {
                grids[row][col].points.clear();
                // cout<<"clear"<<endl;
                continue;
            }
            grids[row][col].x = col * tolerance + 0.5 * tolerance;
            grids[row][col].y = row * tolerance + 0.5 * tolerance;

            if (grids[row][col].points.size() < min_points)
            {
                grids[row][col].cluster = -1;
            }
            else
            {
                grids[row][col].cluster = 0;
            }
        }
    }
    //cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds;
    for (int row = 0; row < grids.size(); row++)
    {
        for (int col = 0; col < grids[row].size(); col++)
        {
            // 筛除个数不合格的
            if (grids[row][col].points.size() >= cloud_2d->points.size() || grids[row][col].points.size() <= 0)
            {
                grids[row][col].points.clear();
                grids[row][col].cluster += 200;
                continue;
            }
            if (grids[row][col].cluster != 0)
                continue;

            grids[row][col].cluster += 200;//　此处已经被访问
            Detected_Obj obj_info;
            std::queue<pcl::PointXYZ> neighbor_queue;
            neighbor_queue.push(pcl::PointXYZ(col, row, 0));
            float obj_minz = grids[row][col].minz;
            float obj_maxz = grids[row][col].maxz;
            pcl::PointCloud<pcl::PointXYZ>::Ptr singlecloud(new pcl::PointCloud<pcl::PointXYZ>);
            while (!neighbor_queue.empty())
            {
                int c_x = neighbor_queue.front().x;
                int c_y = neighbor_queue.front().y;
                singlecloud->insert(singlecloud->points.end(), grids[c_y][c_x].points.begin(), grids[c_y][c_x].points.end());
                neighbor_queue.pop();
                for (int k = -KERNELXSIZE; k <= KERNELXSIZE; k++)
                {
                    for (int l = -KERNELYSIZE; l <= KERNELYSIZE; l++)
                    {
                        if (k == 0 && l == 0)
                            continue;

                        int n_x = c_x + k;
                        int n_y = c_y + l;

                        if (n_x >= 0 && n_y >= 0 && n_y < grids.size() && n_x < grids[0].size())
                        {
                            if (grids[n_y][n_x].cluster == 0)
                            {
                                grids[n_y][n_x].cluster += 200;
                                neighbor_queue.push(pcl::PointXYZ(n_x, n_y, 0));
                            }
                        }
                    }
                }
            }
            pcl_conversions::fromPCL(pointcloud->header, obj_info.bounding_box_.header);
            pointclouds.push_back(singlecloud);
            cloud_size = (int)singlecloud->points.size();
            obj_list->push_back(obj_info);
        }
    }

    if (pointclouds.size() == 0)
    {
        pointcloud->points.clear();
        return;
    }
    int maxlabel = 0;
    int maxSize = -1;
    for (int i = 0; i < pointclouds.size(); ++i)
    {
        if (pointclouds[i]->points.size() > maxSize)
        {
            maxSize = pointclouds[i]->points.size();
            maxlabel = i;
        }
    }
    if (pointclouds[maxlabel]->points.size() > 0)
    {
        pointcloud->points.clear();
        pointcloud->points.insert(pointcloud->points.end(), pointclouds[maxlabel]->points.begin(), pointclouds[maxlabel]->points.end());
    }
    double buildbox_time = (ros::Time::now() - begin_time).toSec();
    // ROS_INFO("%f secs for dbscan (%d clusters).", buildbox_time, cloud_size);
}

} // namespace lidar