#include "mytest/objectdetection_buildminbox.h"
#include <pcl/surface/convex_hull.h>

bool Build_Object(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, jsk_recognition_msgs::BoundingBox &bounding_box_)
{
    ComputeGeometricFeature(cloud, bounding_box_);
    //test
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointXYZ point;
    // for(int i=0;i<40;i++)
    // {
    //     for(int j=0;j<10;j++)
    //     {
    //         point.x = i;
    //         point.y = i+j;
    //         point.z = 10;
    //         pointcloud->push_back(point);
    //     }
    // }
    // ComputeGeometricFeature(pointcloud);
}

void ComputeGeometricFeature(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, jsk_recognition_msgs::BoundingBox &bounding_box_)
{
    // std::vector<std::vector<grid>> grids;
    pcl::PointCloud<pcl::PointXYZ>::Ptr polygons(new pcl::PointCloud<pcl::PointXYZ>);
    // PointCloud2Grid(cloud,grids);
    // std::map<int, int> cell_hash_table;
    // for(int i=0;i<cloud->points.size();i++)
    // {
    //     int row = (int)((cloud->points[i].y-MINY)/CELLSIZEY);
    //     int col = (int)((cloud->points[i].x-MINX)/CELLSIZEX);
    //     cell_hash_table[row * WIDTH+col]++;
    // }
    float obj_height = 0;
    ComputePolygon2dxy(cloud, polygons, obj_height);
    ReconstructPolygon(polygons, bounding_box_, obj_height);
}

// void PointCloud2Grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<std::vector<grid>> & grids)
// {
//     grids.resize((int)((MAXY-MINY)/CELLSIZEY));
//     for(int i=0;i<grids.size();i++)
//     {
//         grids[i].resize((int)((MAXX-MINX)/CELLSIZEX));
//     }
//     pcl::PointCloud<pcl::PointXYZ> cloud_ = *cloud;
//     for(int i=0;i<cloud_.points.size();i++)
//     {
//         int row = (int)((cloud_.points[i].y-MINY)/CELLSIZEY);
//         int col = (int)((cloud_.points[i].x-MINX)/CELLSIZEX);
//         grids[row][col].points.push_back(cloud_.points[i]);
//         grids[row][col].minz = std::numeric_limits<float>::max();
//         grids[row][col].maxz = std::numeric_limits<float>::min();
//     }
//     for(int row=0;row<grids.size();row++)
//     {
//         for(int col=0;col<grids[row].size();col++)
//         {
//             grids[row][col].x = col * CELLSIZEX;
//             grids[row][col].y = row * CELLSIZEY;
//             for(int i=0;i<grids[row][col].points.size();i++)
//             {
//                 if(grids[row][col].maxz<grids[row][col].points[i].z)
//                 {
//                     grids[row][col].maxz = grids[row][col].points[i].z;
//                 }
//                 if(grids[row][col].minz>grids[row][col].points[i].z)
//                 {
//                     grids[row][col].minz = grids[row][col].points[i].z;
//                 }
//             }
//             if(grids[row][col].points.size()<MINPOINTS)
//             {
//                 grids[row][col].cluster = -1;
//             }
//             else
//             {
//                 grids[row][col].cluster = 0;
//             }
//         }
//     }

// }
void ComputePolygon2dxy(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, float &obj_height)
{
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    //get the min x,y,z and max x,y,z
    GetCloudMinMax3D(cloud, &min_pt, &max_pt);
    // cout << "min_pt" << min_pt[0] << " " << min_pt[1] << " " << min_pt[2] << endl;
    // cout << "max_pt" << max_pt[0] << " " << max_pt[1] << " " << max_pt[2] << endl;
    obj_height = max_pt[2] - min_pt[2];
    //    printf("height:%lf\n",obj_height);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointXYZ point;
    // for(int row=0;row<grids.size();row++)
    // {
    //     for(int col=0;col<grids[row].size();col++)
    //     {
    //         if(grids[row][col].cluster==0)
    //         {
    //             point.x = grids[row][col].x;
    //             point.y = grids[row][col].y;
    //             cloud->push_back(point);
    //         }
    //     }
    // }

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud);
    hull.setDimension(2);
    std::vector<pcl::Vertices> polygons_;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> surface_hull;
    hull.reconstruct(surface_hull, polygons_);
    // cout << "print surface_hull" << endl;
    // for (int i = 0; i < surface_hull.size(); i++)
    // {
    //     cout << surface_hull[i].x << " " << surface_hull[i].y << " " << surface_hull[i].z << endl;
    // }

    //if (hull.size() == 1u) {
    if (true)
    {
        for (int k = 0; k < surface_hull.size(); k++)
        {
            // float x = params_.grid_range_max -
            //     (surface_hull[k].y * params_.grid_cell_size)
            //     - params_.grid_cell_size / 2;
            // float y  = params_.grid_range_max -
            //     (surface_hull[k].x * params_.grid_cell_size)
            //     - params_.grid_cell_size / 2;
            // RCLCPP_INFO(nh->get_logger(), "hull x:%f,y:%f",x,y);
            pcl::PointXYZ polygon;
            polygon.x = surface_hull[k].x;
            polygon.y = surface_hull[k].y;
            polygon.z = min_pt[2];
            polygons->push_back(polygon);
        }
    }
    else
    {
        polygons->points.resize(4);
        polygons->points[0].x = static_cast<double>(min_pt[0]);
        polygons->points[0].y = static_cast<double>(min_pt[1]);
        polygons->points[0].z = static_cast<double>(min_pt[2]);

        polygons->points[1].x = static_cast<double>(min_pt[0]);
        polygons->points[1].y = static_cast<double>(max_pt[1]);
        polygons->points[1].z = static_cast<double>(min_pt[2]);

        polygons->points[2].x = static_cast<double>(max_pt[0]);
        polygons->points[2].y = static_cast<double>(max_pt[1]);
        polygons->points[2].z = static_cast<double>(min_pt[2]);

        polygons->points[3].x = static_cast<double>(max_pt[0]);
        polygons->points[3].y = static_cast<double>(min_pt[1]);
        polygons->points[3].z = static_cast<double>(min_pt[2]);
    }
}

void GetCloudMinMax3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f *min_point, Eigen::Vector4f *max_point)
{
    Eigen::Vector4f &min_pt = *min_point;
    Eigen::Vector4f &max_pt = *max_point;
    min_pt[0] = min_pt[1] = min_pt[2] = FLT_MAX;
    max_pt[0] = max_pt[1] = max_pt[2] = -FLT_MAX;
    // for(int row=0;row<grids.size();row++)
    // {
    //     for(int col=0;col<grids[row].size();col++)
    //     {
    //         if(grids[row][col].cluster==0)
    //         {
    //             min_pt[0] = std::min(min_pt[0], grids[row][col].x);
    //             max_pt[0] = std::max(max_pt[0], grids[row][col].x);
    //             min_pt[1] = std::min(min_pt[1], grids[row][col].y);
    //             max_pt[1] = std::max(max_pt[1], grids[row][col].y);
    //             min_pt[2] = std::min(min_pt[2], grids[row][col].minz);
    //             max_pt[2] = std::max(max_pt[2], grids[row][col].maxz);
    //         }
    //     }
    // }
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        min_pt[0] = std::min(min_pt[0], cloud->points[i].x);
        max_pt[0] = std::max(max_pt[0], cloud->points[i].x);
        min_pt[1] = std::min(min_pt[1], cloud->points[i].y);
        max_pt[1] = std::max(max_pt[1], cloud->points[i].y);
        min_pt[2] = std::min(min_pt[2], cloud->points[i].z);
        max_pt[2] = std::max(max_pt[2], cloud->points[i].z);
    }
}

void ReconstructPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, jsk_recognition_msgs::BoundingBox &bounding_box, float &obj_height)
{
    if (polygons->points.size() <= 0)
    {
        return;
    }
    /* 
        求解相对于(0,0)坐标的最左最右点，分别命名为min_point(left),max_point(right)
    */
    size_t max_point_index = 0;
    size_t min_point_index = 0;
    Eigen::Vector3d p;
    p[0] = polygons->points[0].x;
    p[1] = polygons->points[0].y;
    p[2] = polygons->points[0].z;
    Eigen::Vector3d max_point = p;
    Eigen::Vector3d min_point = p;
    for (size_t i = 1; i < polygons->points.size(); ++i)
    {
        Eigen::Vector3d p;
        p[0] = polygons->points[i].x;
        p[1] = polygons->points[i].y;
        p[2] = polygons->points[i].z;
        Eigen::Vector3d ray = p;
        // clock direction
        if (max_point[0] * ray[1] - ray[0] * max_point[1] < EPSILON)
        {
            max_point = ray;
            max_point_index = i;
            //cout<<"process max = "<<i<<endl;
        }
        // unclock direction
        if (min_point[0] * ray[1] - ray[0] * min_point[1] > EPSILON)
        {
            min_point = ray;
            min_point_index = i;
        }
    }
    // cout << "max index = " << max_point_index << endl;
    // cout << "min index = " << min_point_index << endl;

    /* 
        total_len为左右极点以及原点形成区域内的线段之和。
        max_dis为所有线段中的最长线段值。
        has_out代表是否存在背对lidar的点。
    */
    Eigen::Vector3d line = max_point - min_point;
    double total_len = 0;
    double max_dis = 0;
    bool has_out = false;
    for (size_t i = min_point_index, count = 0; count < polygons->points.size(); i = (i + 1) % polygons->points.size(), ++count)
    {
        Eigen::Vector3d p_x;
        p_x[0] = polygons->points[i].x;
        p_x[1] = polygons->points[i].y;
        p_x[2] = polygons->points[i].z;
        size_t j = (i + 1) % polygons->points.size();
        if (j != min_point_index && j != max_point_index)
        {
            Eigen::Vector3d p;
            p[0] = polygons->points[j].x;
            p[1] = polygons->points[j].y;
            p[2] = polygons->points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON)
            {
                double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) +
                                   (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist - max_dis > EPSILON)
                {
                    max_dis = dist;
                }
            }
            else
            {
                // outline
                has_out = true;
            }
        }
        else if ((i == min_point_index && j == max_point_index) || (i == max_point_index && j == min_point_index))
        {
            size_t k = (j + 1) % polygons->points.size();
            Eigen::Vector3d p_k;
            p_k[0] = polygons->points[k].x;
            p_k[1] = polygons->points[k].y;
            p_k[2] = polygons->points[k].z;
            Eigen::Vector3d p_j;
            p_j[0] = polygons->points[j].x;
            p_j[1] = polygons->points[j].y;
            p_j[2] = polygons->points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0)
            {
            }
            else
            {
                // outline
                has_out = true;
            }
        }
        else if (j == min_point_index || j == max_point_index)
        {
            Eigen::Vector3d p;
            p[0] = polygons->points[j].x;
            p[1] = polygons->points[j].y;
            p[2] = polygons->points[j].z;
            Eigen::Vector3d ray = p_x - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < EPSILON)
            {
                double dist = sqrt((p[0] - p_x[0]) * (p[0] - p_x[0]) + (p[1] - p_x[1]) * (p[1] - p_x[1]));
                total_len += dist;
                if (dist > max_dis)
                {
                    max_dis = dist;
                }
            }
            else
            {
                // outline
                has_out = true;
            }
        }
    }
    // cout << "total len = " << total_len << endl;
    // cout << "max dis = " << max_dis << endl;

    size_t count = 0;
    double min_area = std::numeric_limits<double>::max();
    for (size_t i = min_point_index; count < polygons->points.size(); i = (i + 1) % polygons->points.size(), ++count)
    {
        Eigen::Vector3d p_x;
        p_x[0] = polygons->points[i].x;
        p_x[1] = polygons->points[i].y;
        p_x[2] = polygons->points[i].z;
        size_t j = (i + 1) % polygons->points.size();
        Eigen::Vector3d p_j;
        p_j[0] = polygons->points[j].x;
        p_j[1] = polygons->points[j].y;
        p_j[2] = polygons->points[j].z;
        double dist = sqrt((p_x[0] - p_j[0]) * (p_x[0] - p_j[0]) + (p_x[1] - p_j[1]) * (p_x[1] - p_j[1]));
        if (dist < max_dis && (dist / total_len) < 0.5)
        {
            //cout << "continue:" << total_len << endl;
            continue;
        }
        if (j != min_point_index && j != max_point_index)
        {
            Eigen::Vector3d p;
            p[0] = polygons->points[j].x;
            p[1] = polygons->points[j].y;
            p[2] = polygons->points[j].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0)
            {
                Eigen::Vector3d center;
                double length = 0;
                double width = 0;
                Eigen::Vector3d dir;
                double area = ComputeAreaAlongOneEdge(polygons, i, &center, &length, &width, &dir);
                if (area < min_area)
                {
                    bounding_box.pose.position.x = center[0];
                    bounding_box.pose.position.y = center[1];
                    bounding_box.pose.position.z = polygons->points[0].z;
                    bounding_box.dimensions.x = length;
                    bounding_box.dimensions.y = width;
                    bounding_box.dimensions.z = obj_height;
                    Quaternion q = ToQuaternion((atan(dir[1] / dir[0]) / M_PI), 0, 0);
                    bounding_box.pose.orientation.x = q.x;
                    bounding_box.pose.orientation.y = q.y;
                    bounding_box.pose.orientation.z = q.z;
                    bounding_box.pose.orientation.w = q.w;
                    // obj.geometric.orientation = (atan(dir[1]/dir[0])/M_PI) * 180;
                    // dir.normalize();
                    //RCLCPP_INFO(nh->get_logger(), "the dir are %f,%f,%f",dir[0],dir[1],dir[2]);
                    min_area = area;
                }
            }
            else
            {
                // outline
            }
        }
        else if ((i == min_point_index && j == max_point_index) || (i == max_point_index && j == min_point_index))
        {
            if (!has_out)
            {
                continue;
            }
            Eigen::Vector3d center;
            double length = 0;
            double width = 0;
            Eigen::Vector3d dir;
            double area = ComputeAreaAlongOneEdge(polygons, i, &center, &length, &width, &dir);
            if (area < min_area)
            {
                bounding_box.pose.position.x = center[0];
                bounding_box.pose.position.y = center[1];
                bounding_box.pose.position.z = polygons->points[0].z;
                bounding_box.dimensions.x = length;
                bounding_box.dimensions.y = width;
                bounding_box.dimensions.z = obj_height;
                Quaternion q = ToQuaternion((atan(dir[1] / dir[0]) / M_PI), 0, 0);
                bounding_box.pose.orientation.x = q.x;
                bounding_box.pose.orientation.y = q.y;
                bounding_box.pose.orientation.z = q.z;
                bounding_box.pose.orientation.w = q.w;
                // obj.geometric.orientation = (atan(dir[1]/dir[0])/M_PI) * 180;
                // dir.normalize();
                //RCLCPP_INFO(nh->get_logger(), "the dir are %f,%f,%f",dir[0],dir[1],dir[2]);
                min_area = area;
            }
        }
        else if (j == min_point_index || j == max_point_index)
        {
            Eigen::Vector3d p;
            p[0] = polygons->points[i].x;
            p[1] = polygons->points[i].y;
            p[2] = polygons->points[i].z;
            Eigen::Vector3d ray = p - min_point;
            if (line[0] * ray[1] - ray[0] * line[1] < 0)
            {
                Eigen::Vector3d center;
                double length = 0.0;
                double width = 0.0;
                Eigen::Vector3d dir;
                double area = ComputeAreaAlongOneEdge(polygons, i, &center, &length, &width, &dir);
                if (area < min_area)
                {
                    bounding_box.pose.position.x = center[0];
                    bounding_box.pose.position.y = center[1];
                    bounding_box.pose.position.z = polygons->points[0].z;
                    bounding_box.dimensions.x = length;
                    bounding_box.dimensions.y = width;
                    bounding_box.dimensions.z = obj_height;
                    Quaternion q = ToQuaternion((atan(dir[1] / dir[0]) / M_PI), 0, 0);
                    bounding_box.pose.orientation.x = q.x;
                    bounding_box.pose.orientation.y = q.y;
                    bounding_box.pose.orientation.z = q.z;
                    bounding_box.pose.orientation.w = q.w;
                    // obj.geometric.orientation = (atan(dir[1]/dir[0])/M_PI) * 180;
                    // dir.normalize();
                    //RCLCPP_INFO(nh->get_logger(), "the dir are %f,%f,%f",dir[0],dir[1],dir[2]);
                    min_area = area;
                }
            }
            else
            {
                // outline
            }
        }
    }
    // cout << "min area = " << min_area << endl;
    //obj->direction.normalize();
}

double ComputeAreaAlongOneEdge(pcl::PointCloud<pcl::PointXYZ>::Ptr polygons, size_t first_in_point, Eigen::Vector3d *center, double *lenth, double *width, Eigen::Vector3d *dir)
{
    std::vector<Eigen::Vector3d> ns;
    Eigen::Vector3d v(0.0, 0.0, 0.0);
    Eigen::Vector3d vn(0.0, 0.0, 0.0);
    Eigen::Vector3d n(0.0, 0.0, 0.0);
    double len = 0;
    double wid = 0;
    size_t index = (first_in_point + 1) % polygons->points.size();
    for (size_t i = 0; i < polygons->points.size(); ++i)
    {
        if (i != first_in_point && i != index)
        {
            // compute v
            Eigen::Vector3d o(0.0, 0.0, 0.0);
            Eigen::Vector3d a(0.0, 0.0, 0.0);
            Eigen::Vector3d b(0.0, 0.0, 0.0);
            o[0] = polygons->points[i].x;
            o[1] = polygons->points[i].y;
            o[2] = 0;
            b[0] = polygons->points[first_in_point].x;
            b[1] = polygons->points[first_in_point].y;
            b[2] = 0;
            a[0] = polygons->points[index].x;
            a[1] = polygons->points[index].y;
            a[2] = 0;
            double k = ((a[0] - o[0]) * (b[0] - a[0]) + (a[1] - o[1]) * (b[1] - a[1]));
            k = k / ((b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]));
            k = k * -1;
            // n is pedal of src
            n[0] = (b[0] - a[0]) * k + a[0];
            n[1] = (b[1] - a[1]) * k + a[1];
            n[2] = 0;
            // compute height from src to line
            Eigen::Vector3d edge1 = o - b;
            Eigen::Vector3d edge2 = a - b;
            // cross product
            // 叉乘出平行四边形面积，除以底得出高，即为投影距离
            double height = fabs(edge1[0] * edge2[1] - edge2[0] * edge1[1]);
            height = height / sqrt(edge2[0] * edge2[0] + edge2[1] * edge2[1]);

            // cout << "当前i=" << i << endl;
            // cout << "n = " << n[0] << " " << n[1] << " " << n[2] << endl;
            // cout << "edge1 = " << edge1[0] << " " << edge1[1] << " " << edge1[2] << endl;
            // cout << "edge2 = " << edge2[0] << " " << edge2[1] << " " << edge2[2] << endl;
            // cout << "height = " << height << endl;

            if (height > wid) // 存取最大距离设为width
            {
                // cout << "找到一个极值" << endl;
                // cout << "first point = " << first_in_point << " " << i << endl;
                // cout << "o = " << o << endl;
                wid = height;
                v = o;
                vn = n;
            }
        }
        else
        {
            n[0] = polygons->points[i].x;
            n[1] = polygons->points[i].y;
            n[2] = 0;
        }
        ns.push_back(n);
    }
    // 找出投影在直线后两点距离的最大值
    size_t point_num1 = 0;
    size_t point_num2 = 0;
    for (size_t i = 0; i < ns.size() - 1; ++i)
    {
        Eigen::Vector3d p1 = ns[i];
        for (size_t j = i + 1; j < ns.size(); ++j)
        {
            Eigen::Vector3d p2 = ns[j];
            double dist = sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]));
            if (dist > len)
            {
                len = dist;
                point_num1 = i;
                point_num2 = j;
            }
        }
    }
    // cout << "最大dist两点的index = " << point_num1 << " " << point_num2 << endl;

    Eigen::Vector3d vp1 = v + ns[point_num1] - vn;
    Eigen::Vector3d vp2 = v + ns[point_num2] - vn;
    (*center) = (vp1 + vp2 + ns[point_num1] + ns[point_num2]) / 4;
    (*center)[2] = polygons->points[0].z;
    // cout << "V = " << v << endl;
    // cout << "ns[point_num1]" << ns[point_num1] << endl;
    // cout << "ns[point_num2]" << ns[point_num2] << endl;
    // cout << "VN = " << vn << endl;
    // cout << "VP1 = " << vp1 << endl;
    // cout << "VP2 = " << vp2 << endl;
    // cout << "center = " << center[0] << " " << center[1] << endl;

    if (len > wid)
    {
        *dir = ns[point_num2] - ns[point_num1];
    }
    else
    {
        *dir = vp1 - ns[point_num1];
    }
    *lenth = len > wid ? len : wid;
    *width = len > wid ? wid : len;
    return (*lenth) * (*width);
}

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

void Quaternion2Euler(const Quaternion &q, double &yaw, double &pitch, double &roll)
{
    yaw = std::atan(2 * (q.x * q.y - q.w * q.z) / (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
    pitch = std::asin(-2 * (q.w * q.y + q.x * q.z));
    roll = std::atan(2 * (q.y * q.z - q.w * q.x) / (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}
