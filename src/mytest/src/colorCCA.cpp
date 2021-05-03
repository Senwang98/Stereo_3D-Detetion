#include "mytest/colorCCA.h"
#include "mytest/dbscan.h"
#include <opencv2/xfeatures2d.hpp>
#include <pcl/features/normal_3d.h>

DECLARE_int32(threshold_similar);

colorCCA::colorCCA(cv::Mat &input_img, cv::Mat &input_label,
                   double b_in, double f_in, double resx_in, double resz_in, double u0, double v0, cv::Mat &src_img)
{
    src = &src_img;
    disparity_img = &input_img;
    idxImg = &input_label;
    threshold_similar = FLAGS_threshold_similar;
    b = b_in;
    f = f_in;
    resx = resx_in;
    resz = resz_in;
    U0 = u0;
    V0 = v0;
    inner = (cv::Mat_<float>(3, 3) << f, 0, 640,
             0, f, 360,
             0, 0, 1);
    outer = (cv::Mat_<float>(3, 4) << cos(0.05), 0, sin(0.05), 0,
             0, 1, 0, 0,
             -sin(0.05), 0, cos(0.05), 1600);
}

cv::Vec3b colorCCA::make_color(int id)
{
    static int color_array[][3] = {
        {255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 218, 185}, {47, 79, 79}, {100, 149, 237}, {0, 191, 255}, {64, 224, 208}, {102, 205, 170}, {124, 252, 0}, {205, 92, 92}, {255, 20, 147}, {160, 32, 240}};
    int icolor = id % 16;
    return cv::Vec3b(color_array[icolor][0], color_array[icolor][1], color_array[icolor][2]);
}

bool colorCCA::dist(int r1, int c1, int r2, int c2)
{
    bool similar = false;
    if (r2 >= 0)
    {
        float delta = abs(disparity_img->at<float>(r1, c1) - disparity_img->at<float>(r2, c2));
        similar = delta < ((float)threshold_similar * disparity_img->at<float>(r2, c2) / 20.0 + 3);
    }
    return similar;
}

bool colorCCA::colorsegment_seedfill(int border)
{
    cout << "相似度为：" << threshold_similar << endl;
    if (disparity_img->empty() || disparity_img->channels() != 1)
    {
        return false;
    }

    int width = disparity_img->cols;
    int height = disparity_img->rows;
    label_count.push_back(0); // for class 0
    label_count.push_back(0); // for class 1

    for (int y = 0; y < idxImg->rows; ++y)
    {
        for (int x = 0; x < idxImg->cols; ++x)
        {
            if (idxImg->at<float>(y, x) >= 3.0)
            {
                idxImg->at<float>(y, x) = 1;
            }
            else
            {
                idxImg->at<float>(y, x) = 0;
            }
        }
    }

    int label = 1; // class 1 is not used
    int count = 0;
    for (int i = border; i < height - border; ++i)
    {
        float *curRow = idxImg->ptr<float>(i);
        for (int j = border; j < width - border; ++j)
        {
            float *cur_data = curRow + j;

            if (*cur_data == 0)
            {
                continue;
            }
            if (*cur_data == 1)
            {
                count = 0;
                count++;
                *cur_data = ++label; // class 1 is not used

                std::stack<std::pair<int, int>> neighbor_data;
                if (j < width - 1 && (idxImg->at<float>(i, j + 1) == 1))
                {
                    neighbor_data.push(std::pair<int, int>(i, j + 1)); // right
                }
                if (i < height - 1 && (idxImg->at<float>(i + 1, j) == 1))
                {
                    neighbor_data.push(std::pair<int, int>(i + 1, j)); // down
                }

                while (!neighbor_data.empty())
                {
                    std::pair<int, int> position = neighbor_data.top();
                    int cur_y = position.first;
                    int cur_x = position.second;

                    if (cur_x <= 0 || cur_x >= width - 1 || cur_y <= 0 || cur_y >= height - 1)
                    {
                        neighbor_data.pop(); // 注意弹出旧值
                        continue;
                    }
                    float &label_data = idxImg->ptr<float>(cur_y)[cur_x];

                    // color disparity_img has no value
                    if (0 == label_data)
                    {
                        neighbor_data.pop(); // 注意弹出旧值
                        continue;
                    }
                    neighbor_data.pop();
                    if (dist(cur_y, cur_x, i, j))
                    {
                        label_data = label;
                        count++;
                        // up
                        if ((cur_y >= 1) && idxImg->ptr<float>(cur_y - 1)[cur_x] == 1)
                        {
                            neighbor_data.push(std::pair<int, int>(cur_y - 1, cur_x));
                        }
                        // down
                        if (idxImg->ptr<float>(cur_y + 1)[cur_x] == 1)
                        {
                            neighbor_data.push(std::pair<int, int>(cur_y + 1, cur_x));
                        }
                        // left
                        if ((cur_x >= 1) && idxImg->ptr<float>(cur_y)[cur_x - 1] == 1)
                        {
                            neighbor_data.push(std::pair<int, int>(cur_y, cur_x - 1));
                        }
                        // right
                        if (idxImg->ptr<float>(cur_y)[cur_x + 1] == 1)
                        {
                            neighbor_data.push(std::pair<int, int>(cur_y, cur_x + 1));
                        }
                        //  // up
                        // if ((cur_y >= 1) && idxImg->ptr<float>(cur_y - 1)[cur_x] == 1)
                        // {
                        //     neighbor_data.push(std::pair<int, int>(cur_y - 1, cur_x));
                        // }
                        // // down
                        // if (idxImg->ptr<float>(cur_y + 1)[cur_x] == 1)
                        // {
                        //     neighbor_data.push(std::pair<int, int>(cur_y + 1, cur_x));
                        // }
                        // // left
                        // if ((cur_x >= 1) && idxImg->ptr<float>(cur_y)[cur_x - 1] == 1)
                        // {
                        //     neighbor_data.push(std::pair<int, int>(cur_y, cur_x - 1));
                        // }
                        // // right
                        // if (idxImg->ptr<float>(cur_y)[cur_x + 1] == 1)
                        // {
                        //     neighbor_data.push(std::pair<int, int>(cur_y, cur_x + 1));
                        // }
                    }
                }
                // when pixel number of a segment is less than 200, ignore them
                label_count.push_back(count > 500 ? count : 0);
            }
        }
    }
    return true;
}

void colorCCA::getBbox(int number)
{
    // 根据CCA结果预设空间
    for (int i = 0; i < label_count.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        obj_pcls.push_back(tmp);
    }

    std::vector<vector<int>> mp(400); //记录像素是否被投影
    pcl::PointCloud<pcl::PointXYZ>::Ptr dbscan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < mp.size(); i++)
    {
        mp[i].clear();
        mp[i].resize(600);
        for (int j = 0; j < mp[i].size(); j++)
            mp[i][j] = 0;
    }

    cv::Mat show_cloud(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));

    int cnt = 0;

    for (int y = 0; y < idxImg->rows; ++y)
    {
        for (int x = 0; x < idxImg->cols; ++x)
        {
            if (idxImg->at<float>(y, x) != 0 && idxImg->at<float>(y, x) != 1 && label_count[idxImg->at<float>(y, x)] != 0)
            {
                double tempx = ((x - U0) * b / 1000) / (disparity_img->at<float>(y, x) * resx); // x(width) in camera coordinate (m)
                double tempz = (f * b / 1000) / (disparity_img->at<float>(y, x) * resz);        // z(depth) in camera coordinate (m)
                double tempy = ((y - V0) * b / 1000) / disparity_img->at<float>(y, x);          // y(height) in camera coordinate (m)

                pcl::PointXYZ tmp_p = pcl::PointXYZ(tempx, tempz, tempy); // tempx -> x, tempy -> z, tempz -> y
                obj_pcls[(int)idxImg->at<float>(y, x)]->points.push_back(tmp_p);
                cnt++;
                show_cloud.at<cv::Vec3b>(400 - (int)(tempz), (int)(tempx + 300)) = make_color((int)idxImg->at<float>(y, x));
                if (mp[400 - (int)(tempz)][(int)(tempx + 300)] == 0)
                {
                    mp[400 - (int)(tempz)][(int)(tempx + 300)] = 1;
                    pcl::PointXYZ p = pcl::PointXYZ((int(tempx) + 300), (400 - int(tempz)), int(0));
                    dbscan_cloud->points.push_back(p);
                }
                else
                {
                    continue;
                }
            }
        }
    }

    cv::imshow("grid", show_cloud);

    // 设置去地面测试点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mytest(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < obj_pcls.size(); i++)
    {
        for (int j = 0; j < obj_pcls[i]->points.size(); j++)
        {
            cloud_mytest->points.push_back(obj_pcls[i]->points[j]);
        }
    }

    //cout << "dbscan之前点云个数为：" << cnt << endl;
    double r = 5;
    int minPts = 25;
    //cv::Mat show_mp(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    //cout<<"dbscan之前像素点个数为："<<dbscan_cloud->points.size()<<endl;
    // for(int i=0;i<dbscan_cloud->points.size();i++)
    // {
    //     show_mp.at<cv::Vec3b>(int(dbscan_cloud->points[i].y),int(dbscan_cloud->points[i].x))= make_color((int)idxImg->at<float>(i, i));
    // }
    //cv::imshow("show_mp",show_mp);
    DBSCAN dbScan(dbscan_cloud->points.size(), r, minPts);
    dbScan.run(dbscan_cloud);
    /*
        此处得到cluster的过程中，人为设定了允许分类存在的最小阈值
        如果低于该阈值，则dbscan将认定为噪点并删除该分类
    */
    vector<vector<point>> cluster = dbScan.get_cluster();

    // 根据cluster展示效果图
    // cv::Mat cluster_show(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));

    // for(int i=0;i<cluster.size();i++)
    // {
    //     for(int j=0;j<cluster[i].size();j++)
    //     {
    //         cluster_show.at<cv::Vec3b>(int(cluster[i][j].z), int(cluster[i][j].x)) = colorCCA::make_color(i);
    //     }
    // }
    // cv::imshow("cluster show",cluster_show);
    // cv::waitKey(0);

    //　此时的dbscan_cloud保存的是二维点
    int cluster_cnt = 0;
    for (int i = 0; i < cluster.size(); i++)
        if (cluster[i].size() > 0)
            cluster_cnt++;
    label_count.clear();
    label_count.resize(cluster_cnt);
    // cout << "修改label_count成功！" << endl;

    // //cout<<"dbscan之后像素点个数为："<<dbscan_cloud->points.size()<<endl;
    // // cv::Mat show_mp1(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    // // for(int i=0;i<dbscan_cloud->points.size();i++)
    // // {
    // //     show_mp1.at<cv::Vec3b>(int(dbscan_cloud->points[i].y),int(dbscan_cloud->points[i].x))= make_color((int)idxImg->at<float>(i, i));
    // // }
    // // cv::imshow("show_mp1",show_mp1);
    for (int i = 0; i < mp.size(); i++) // 现在有聚类之后的点，mp标志清零，在mp上更新聚类结果
    {
        for (int j = 0; j < mp[i].size(); j++)
            mp[i][j] = -1;
    }

    int c = 0;
    // cout << "cluster.size()=" << cluster.size() << endl;
    for (int i = 0; i < cluster.size(); i++) //更新mp
    {
        cout << "cluster[i].size()=" << cluster[i].size() << endl;
        for (int j = 0; j < cluster[i].size(); j++)
        {
            int col = cluster[i][j].x;
            int row = cluster[i][j].z;
            mp[row][col] = i;
            c++;
            //cout << "mp[" << row << "][" << col << "] = " << mp[row][col] << endl;
        }
    }
    // cout<<"cluster.szie()="<<c<<endl;
    // cout << "mp更新成功！" << endl;

    int test_cnt = 0;
    //根据mp筛选点云
    int erase_cnt = 0;
    int exist_cnt = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp_pcl;
    for (int i = 0; i < label_count.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        tmp_pcl.push_back(tmp);
    }
    for (int i = 0; i < obj_pcls.size(); i++)
    {
        for (int j = 0; j < obj_pcls[i]->points.size(); j++)
        {
            test_cnt++;
            int x = int(obj_pcls[i]->points[j].x) + 300;
            int y = 400 - int(obj_pcls[i]->points[j].y);
            // if (mp[y][x] == -1)
            // {
            //     pcl::PointCloud<pcl::PointXYZ>::iterator index = obj_pcls[i]->begin();
            //     obj_pcls[i]->erase(index + j);
            //     // if(j!=0)
            //     j--;
            //     erase_cnt++;
            // }
            // else
            if (mp[y][x] != -1)
            {
                label_count[mp[y][x]]++;
                tmp_pcl[mp[y][x]]->points.push_back(obj_pcls[i]->points[j]);
                exist_cnt++;
            }
        }
    }
    // cout << "筛选中统计的点云个数为：" << test_cnt << endl;
    // cout << "删除了点云数为：" << erase_cnt << "存在的点云个数：" << exist_cnt << endl;
    cv::Mat show_dbscan(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    // cout << "new cloud ok " << endl;

    // 现在的tmp_pcl存放了过滤点云数据，清空obj_pcls
    for (int i = 0; i < obj_pcls.size(); i++)
        obj_pcls[i]->clear();
    obj_pcls.clear();
    obj_pcls.resize(0);
    for (int i = 0; i < label_count.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        obj_pcls.push_back(tmp);
    }
    // 给obj_pcls重新赋值
    for (int i = 0; i < tmp_pcl.size(); i++)
    {
        for (int j = 0; j < tmp_pcl[i]->points.size(); j++)
        {
            obj_pcls[i]->points.push_back(tmp_pcl[i]->points[j]);
        }
    }
    cv::Mat dbscan_test(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        if (label_count[i] != 0)
        {
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                dbscan_test.at<cv::Vec3b>(400 - (int)(obj_pcls[i]->points[j].y), (int)(obj_pcls[i]->points[j].x + 300)) = colorCCA::make_color(i);
            }
        }
    }
    cv::imshow("dbscan", dbscan_test);
    // cv::waitKey(0);

    // 计算点云边界
    std::vector<std::vector<double>> Box_cloud; // 6 numbers: minX, minY, maxX, maxY, centerX, centerY

    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        if (label_count[i] != 0)
        {
            double sumX = 0.0, sumY = 0.0;
            double maxX = -std::numeric_limits<double>::max();
            double minX = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                sumX += obj_pcls[i]->points[j].x;
                sumY += obj_pcls[i]->points[j].y;
                if (maxX < obj_pcls[i]->points[j].x)
                    maxX = obj_pcls[i]->points[j].x;
                if (minX > obj_pcls[i]->points[j].x)
                    minX = obj_pcls[i]->points[j].x;
                if (maxY < obj_pcls[i]->points[j].y)
                    maxY = obj_pcls[i]->points[j].y;
                if (minY > obj_pcls[i]->points[j].y)
                    minY = obj_pcls[i]->points[j].y;
            }
            std::vector<double> min_temp(6);
            min_temp[0] = minX;
            min_temp[1] = minY;
            min_temp[2] = maxX;
            min_temp[3] = maxY;
            min_temp[4] = sumX / obj_pcls[i]->points.size();
            min_temp[5] = sumY / obj_pcls[i]->points.size();
            Box_cloud.push_back(min_temp);
        }
        else
        {
            std::vector<double> min_temp(6);
            Box_cloud.push_back(min_temp);
        }
    }

    // 伪dbscan
    cv::Mat per_dbscan_test(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        if (label_count[i] != 0)
        {
            if (obj_pcls[i]->points.size() > 0)
            {
                // std::cout << "size before = " << obj_pcls[i]->points.size() << std::endl;
                boost::shared_ptr<std::vector<lidar::Detected_Obj>> det_obj(new std::vector<lidar::Detected_Obj>);
                lidar::ObjectDetection_Dbscan dbscan;
                dbscan.Cluster_Object(obj_pcls[i], Box_cloud[i], 1,
                                      (int)std::max(Box_cloud[i][2] - Box_cloud[i][0], Box_cloud[i][3] - Box_cloud[i][1]) / 2, det_obj);

                if (obj_pcls[i]->points.size() == 0)
                {
                    label_count[i] = 0;
                }
                // std::cout << "size after = " << obj_pcls[i]->points.size() << std::endl;
            }
            else
            {
                continue;
            }
        }
    }
    for (int i = 0; i < obj_pcls.size(); ++i)
        if (label_count[i] != 0)
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
                per_dbscan_test.at<cv::Vec3b>(400 - int(obj_pcls[i]->points[j].y), int(obj_pcls[i]->points[j].x) + 300) = cv::Vec3b(255, 255, 255);

    // 显示伪dbscan结果
    cv::imshow("per_dbscan_test", per_dbscan_test);

    // 重新打表,1代表这个像素可以，无需删除
    for (int i = 0; i < mp.size(); i++)
        for (int j = 0; j < mp[i].size(); j++)
            mp[i][j] = 1;

    // dbscan_test存储了现在的投影图
    for (int i = 0; i < per_dbscan_test.rows; i++)
    {
        for (int j = 0; j < per_dbscan_test.cols; j++)
        {
            if (per_dbscan_test.at<Vec3b>(i, j) != cv::Vec3b(0, 0, 0)) // 找到一个投影点
            {
                int neighbour = 0;
                for (int k = -2; k <= 2; k++) //row
                {
                    for (int l = -2; l <= 2; l++) //col
                    {
                        int xx = i + k, yy = j + l;
                        if (xx >= 0 && xx < per_dbscan_test.rows && yy >= 0 && yy < per_dbscan_test.cols)
                        {
                            if (per_dbscan_test.at<cv::Vec3b>(xx, yy) != cv::Vec3b(0, 0, 0))
                                neighbour++;
                        }
                    }
                }
                if (neighbour <= 3)
                {
                    // 相邻点数过低，认为该投影点为噪点
                    mp[i][j] = 0;
                }
            }
        }
    }

    // 此时mp被更新
    int del_cnt = 0;
    for (int i = 0; i < obj_pcls.size(); i++)
    {
        if (label_count[i] != 0)
        {
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                int x = int(obj_pcls[i]->points[j].x) + 300;
                int y = 400 - int(obj_pcls[i]->points[j].y);
                if (mp[y][x] == 0)
                {
                    pcl::PointCloud<pcl::PointXYZ>::iterator index = obj_pcls[i]->begin();
                    obj_pcls[i]->erase(index + j);
                    j--;
                    del_cnt++;
                    label_count[i]--;
                }
            }
        }
    }
    cout << "删除点数:" << del_cnt << endl;
    cout << "再次展示投影图！" << endl;
    cv::Mat del_again_test(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < obj_pcls.size(); ++i)
        if (label_count[i] != 0)
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
                del_again_test.at<cv::Vec3b>(400 - int(obj_pcls[i]->points[j].y), int(obj_pcls[i]->points[j].x) + 300) = colorCCA::make_color(i);
    cv::imshow("del_again_test", del_again_test);

    // 点云融合
    cv::Mat cloud_merge(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < Box_cloud.size(); i++)
        Box_cloud.clear();
    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        if (label_count[i] != 0)
        {
            double sumX = 0.0, sumY = 0.0;
            double maxX = -std::numeric_limits<double>::max();
            double minX = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                sumX += obj_pcls[i]->points[j].x;
                sumY += obj_pcls[i]->points[j].y;
                if (maxX < obj_pcls[i]->points[j].x)
                    maxX = obj_pcls[i]->points[j].x;
                if (minX > obj_pcls[i]->points[j].x)
                    minX = obj_pcls[i]->points[j].x;
                if (maxY < obj_pcls[i]->points[j].y)
                    maxY = obj_pcls[i]->points[j].y;
                if (minY > obj_pcls[i]->points[j].y)
                    minY = obj_pcls[i]->points[j].y;
            }
            std::vector<double> min_temp(6);
            min_temp[0] = minX;
            min_temp[1] = minY;
            min_temp[2] = maxX;
            min_temp[3] = maxY;
            min_temp[4] = sumX / obj_pcls[i]->points.size();
            min_temp[5] = sumY / obj_pcls[i]->points.size();
            Box_cloud.push_back(min_temp);
        }
        else
        {
            std::vector<double> min_temp(6);
            Box_cloud.push_back(min_temp);
        }
    }

    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        if (label_count[i] != 0)
        {
            for (int j = i + 1; j < obj_pcls.size(); ++j)
            {
                double dis = sqrt((Box_cloud[i][4] - Box_cloud[j][4]) * (Box_cloud[i][4] - Box_cloud[j][4]) +
                                  (Box_cloud[i][5] - Box_cloud[j][5]) * (Box_cloud[i][5] - Box_cloud[j][5]));
                if (abs(Box_cloud[i][4] - Box_cloud[j][4]) + abs(Box_cloud[i][5] - Box_cloud[j][5]) <= 25)
                {
                    //std::cout << "before merge, i = " << obj_pcls[i]->points.size() << ", j=" << obj_pcls[j]->points.size() << endl;
                    *obj_pcls[i] += *obj_pcls[j];
                    label_count[i] += label_count[j];
                    label_count[j] = 0;
                    //std::cout << "combine size = " << obj_pcls[i]->points.size() << std::endl;
                }
            }
        }
    }

    for (int i = 0; i < obj_pcls.size(); ++i)
        if (label_count[i] != 0)
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
                cloud_merge.at<cv::Vec3b>(400 - (int)(obj_pcls[i]->points[j].y), (int)(obj_pcls[i]->points[j].x + 300)) = colorCCA::make_color(i);

    cv::imshow("cloud_merge", cloud_merge);

    // 计算出底面的四个点坐标
    int draw_box_cnt = 0;
    vector<vector<Point>> point8s;
    for (int i = 0; i < obj_pcls.size(); ++i)
    {
        // if(i==0)
        if (label_count[i] != 0 && obj_pcls[i]->points.size() >= 100)
        {
            draw_box_cnt++;
            cv::Mat tmp(cv::Size(600, 400), CV_8UC3, cv::Scalar(0, 0, 0));

            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                tmp.at<cv::Vec3b>(400 - (int)(obj_pcls[i]->points[j].y), (int)(obj_pcls[i]->points[j].x + 300)) = cv::Vec3b(255, 255, 255);
            }
            /*      the bbx.pose.position in x-z and x-y plane is located in
             *     Z^    ...........                  Y^ ...........
             *      |    .         .                   | .         .
             *      |    .    *    .                   | .    *    .
             *      |    .         .                   | .         .
             *      |    ...........  -------> X       | ........... -------> X
             * */
            double maxX = -std::numeric_limits<double>::max();
            double minX = std::numeric_limits<double>::max();
            double maxY = -std::numeric_limits<double>::max();
            double minY = std::numeric_limits<double>::max();
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                if (maxX < obj_pcls[i]->points[j].x)
                    maxX = obj_pcls[i]->points[j].x;
                if (minX > obj_pcls[i]->points[j].x)
                    minX = obj_pcls[i]->points[j].x;
                if (maxY < obj_pcls[i]->points[j].y)
                    maxY = obj_pcls[i]->points[j].y;
                if (minY > obj_pcls[i]->points[j].y)
                    minY = obj_pcls[i]->points[j].y;
            }
            jsk_recognition_msgs::BoundingBox bbx;

            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                obj_pcls[i]->points[j].x = obj_pcls[i]->points[j].x - 300;
                obj_pcls[i]->points[j].x *= -1;
                obj_pcls[i]->points[j].y = 400 - obj_pcls[i]->points[j].y;
            }
            // Apollo min box
            Build_Object(obj_pcls[i], bbx);
            if ((maxY - minY) < (maxX - minX))
            {
                swap(bbx.dimensions.x, bbx.dimensions.y);
            }
            Quaternion q;
            q.x = bbx.pose.orientation.x;
            q.y = bbx.pose.orientation.y;
            q.z = bbx.pose.orientation.z;
            q.w = bbx.pose.orientation.w;
            double yaw, pitch, roll;
            Quaternion2Euler(q, yaw, pitch, roll);

            cv::Mat data_z = cv::Mat::zeros(obj_pcls[i]->points.size(), 1, CV_64FC1);
            cv::Scalar mean_z;
            cv::Scalar dev_z;
            for (int j = 0; j < obj_pcls[i]->points.size(); ++j)
            {
                obj_pcls[i]->points[j].x /= -1;
                obj_pcls[i]->points[j].x = obj_pcls[i]->points[j].x + 300;
                obj_pcls[i]->points[j].y = 400 - obj_pcls[i]->points[j].y;
                data_z.at<double>(j, 0) = obj_pcls[i]->points[j].z;
            }
            cv::meanStdDev(data_z, mean_z, dev_z);

            // yaw = 0;
            if (abs(yaw) <= 0.5)
            {
                yaw = 0;
                bbx.dimensions.x = maxY - minY;
                bbx.dimensions.y = maxX - minX;
                bbx.pose.position.x = 300 - (minX + (maxX - minX) / 2);
                bbx.pose.position.y = 400 - (minY + (maxY - minY) / 2);
            }
            yaws.push_back(yaw);
            bbx.pose.position.x *= -1;
            bbx.pose.position.x += 300;
            bbx.pose.position.y = 400 - bbx.pose.position.y;
            bbx.pose.position.z = mean_z[0];
            bbx.dimensions.x /= 2;

            // /* here dimension.x is the long side, dimension.y is the short side, so the formula is changed
            //  *              X^
            //  *               |
            //  *               |
            //  *    Y <--------|  yaw is the angle of point cloud's angle;
            //  *                  when yaw > 0, yaw is in the left side of X and the value is the angle to X
            //  * */

            double miniY = (bbx.dimensions.y - 2 * bbx.dimensions.x * tan(0.08));

            // cv::Point leftup(317, 224);
            // cv::Point rightup(359, 224);
            // cv::Point rightdown(359, 247);
            // cv::Point leftdown(317, 247);

            cv::Point leftup((int)(bbx.pose.position.x - bbx.dimensions.y * cos(yaw) / 2 - bbx.dimensions.x * sin(yaw) / 2) + 300,
                             400 - (int)(bbx.pose.position.y - bbx.dimensions.y * sin(yaw) / 2 + bbx.dimensions.x * cos(yaw) / 2));
            cv::Point rightup((int)(bbx.pose.position.x + bbx.dimensions.y * cos(yaw) / 2 - bbx.dimensions.x * sin(yaw) / 2) + 300,
                              400 - (int)(bbx.pose.position.y + bbx.dimensions.y * sin(yaw) / 2 + bbx.dimensions.x * cos(yaw) / 2));
            cv::Point rightdown((int)(bbx.pose.position.x + bbx.dimensions.y * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
                                400 - (int)(bbx.pose.position.y + bbx.dimensions.y * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));
            cv::Point leftdown((int)(bbx.pose.position.x - bbx.dimensions.y * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
                               400 - (int)(bbx.pose.position.y - bbx.dimensions.y * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));
            vector<Point> t;
            t.push_back(leftup);
            t.push_back(rightup);
            t.push_back(rightdown);
            t.push_back(leftdown);
            point8s.push_back(t);
            // // 先计算出前面的长度
            // //　左上转成UV
            // double d = (f * b) / ((400.0 - leftup.y) * resz);
            // double u1 = (int)(((leftup.x - 300.0) * d * resx) / b + U0);
            // double v1 = (int)(((bbx.pose.position.z + bbx.dimensions.z / 2) * d) / b + V0);

            // // 右上转成UV
            // d = (f * b) / ((400.0 - rightup.y) * resz);
            // double u3 = (int)(((rightup.x - 300.0) * d * resx) / b + U0);
            // double v3 = (int)(((bbx.pose.position.z + bbx.dimensions.z / 2) * d) / b + V0);

            // double dis1 = sqrt((u1 - u3) * (u1 - u3) + (v1 - v3) * (v1 - v3));

            // double min_dis = 1e5;
            // double min_angle = 0;
            // for (double k = 0; k <= atan(bbx.dimensions.y / (2 * bbx.dimensions.x)); k += 0.1)
            // {
            //     miniY = (bbx.dimensions.y - 2 * bbx.dimensions.x * tan(k));
            //     cv::Point rightdown((int)(bbx.pose.position.x + miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
            //                         400 - (int)(bbx.pose.position.y + miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));
            //     cv::Point leftdown((int)(bbx.pose.position.x - miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
            //                        400 - (int)(bbx.pose.position.y - miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));
            //     //　左下转成UV
            //     d = (f * b) / ((400.0 - leftdown.y) * resz);
            //     double u2 = (int)(((leftdown.x - 300.0) * d * resx) / b + U0);
            //     double v2 = (int)(((bbx.pose.position.z + bbx.dimensions.z / 2) * d) / b + V0);
            //     // 右下转成UV
            //     d = (f * b) / ((400.0 - rightdown.y) * resz);
            //     double u4 = (int)(((rightdown.x - 300.0) * d * resx) / b + U0);
            //     double v4 = (int)(((bbx.pose.position.z + bbx.dimensions.z / 2) * d) / b + V0);
            //     double dis2 = sqrt((u2 - u4) * (u2 - u4) + (v2 - v4) * (v2 - v4));
            //     if (dis2 > dis1 && min_dis > dis2 - dis1)
            //     {
            //         min_angle = k;
            //         min_dis = dis2 - dis1;
            //     }
            // }
            // cout << "min_angle = " << min_angle << endl;
            // miniY = (bbx.dimensions.y - 2 * bbx.dimensions.x * tan(min_angle));

            // rightdown.x = (int)(bbx.pose.position.x + miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300;
            // rightdown.y = 400 - (int)(bbx.pose.position.y + miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2);
            // leftdown.x = (int)(bbx.pose.position.x - miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300;
            // leftdown.y = 400 - (int)(bbx.pose.position.y - miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2);

            // cv::Point rightdown((int)(bbx.pose.position.x + miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
            //                     400 - (int)(bbx.pose.position.y + miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));
            // cv::Point leftdown((int)(bbx.pose.position.x - miniY * cos(yaw) / 2 + bbx.dimensions.x * sin(yaw) / 2) + 300,
            //                    400 - (int)(bbx.pose.position.y - miniY * sin(yaw) / 2 - bbx.dimensions.x * cos(yaw) / 2));

            cv::line(del_again_test, leftup, rightup, cv::Scalar(255, 0, 0));
            cv::line(del_again_test, rightup, rightdown, cv::Scalar(0, 0, 255));
            cv::line(del_again_test, rightdown, leftdown, cv::Scalar(0, 255, 0));
            cv::line(del_again_test, leftdown, leftup, cv::Scalar(255, 255, 255));
            BBoxes.push_back(bbx);
            mini_dimensionY.push_back(miniY);
            // cv::imshow("tmp", tmp);
        }
    }
    // cout << "draw_box_cnt = " << draw_box_cnt << endl;
    // // cv::imshow("show_cloud", show_cloud);
    cv::imshow("final_result", del_again_test);

    //// transform 3Dbbox to 2D ////

    BBox2D.resize(BBoxes.size());
    for (int i = 0; i < BBoxes.size(); ++i)
    {
        // if (abs(yaws[i]) <= 2)
        // {
        //     yaws[i] = 0;
        // }
        // cout<<"test:"<<endl;
        // cout << "左上：" << point8s[i][0].y << " " << point8s[i][0].x << endl;
        // cout << "右上：" << point8s[i][1].y << " " << point8s[i][1].x << endl;
        // cout << "右下：" << point8s[i][2].y << " " << point8s[i][2].x << endl;
        // cout << "左下：" << point8s[i][3].y << " " << point8s[i][3].x << endl;
        Point leftup = point8s[i][0];
        Point rightup = point8s[i][1];
        Point leftdown = point8s[i][3];
        Point rightdown = point8s[i][2];
        //此处的ｙ为深度信息
        // frontdownleft
        double d_tmp = (f * b) / ((400 - leftup.y) * resz);
        BBox2D[i].frontdownleft[0] = (int)((leftup.x - 300) * d_tmp * resx) / b + U0;
        BBox2D[i].frontdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // backdownleft
        d_tmp = (f * b) / ((400 - leftdown.y) * resz);
        BBox2D[i].backdownleft[0] = (int)((leftdown.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].backdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // frontdownright
        d_tmp = (f * b) / ((400 - rightup.y) * resz);
        BBox2D[i].frontdownright[0] = (int)((rightup.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].frontdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // backdownright
        d_tmp = (f * b) / ((400 - rightdown.y) * resz);
        BBox2D[i].backdownright[0] = (int)((rightdown.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].backdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        double dis1 = sqrt((BBox2D[i].frontdownleft[0] - BBox2D[i].frontdownright[0]) * (BBox2D[i].frontdownleft[0] - BBox2D[i].frontdownright[0]) +
                           (BBox2D[i].frontdownleft[1] - BBox2D[i].frontdownright[1]) * (BBox2D[i].frontdownleft[1] - BBox2D[i].frontdownright[1]));
        std::cout << "输出前框宽度为：" << dis1 << std::endl;

        // frontupleft
        d_tmp = (f * b) / ((400 - leftup.y) * resz);
        BBox2D[i].frontupleft[0] = (int)((leftup.x - 300) * d_tmp * resx) / b + U0;
        BBox2D[i].frontupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // backupleft
        d_tmp = (f * b) / ((400 - leftdown.y) * resz);
        BBox2D[i].backupleft[0] = (int)((leftdown.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].backupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // frontupright
        d_tmp = (f * b) / ((400 - rightup.y) * resz);
        BBox2D[i].frontupright[0] = (int)((rightup.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].frontupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        // backupright
        d_tmp = (f * b) / ((400 - rightdown.y) * resz);
        BBox2D[i].backupright[0] = (int)((rightdown.x - 300) * d_tmp * resx / b + U0);
        BBox2D[i].backupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

        double dis2 = sqrt((BBox2D[i].backdownleft[0] - BBox2D[i].backdownright[0]) * (BBox2D[i].backdownleft[0] - BBox2D[i].backdownright[0]) +
                           (BBox2D[i].backdownleft[1] - BBox2D[i].backdownright[1]) * (BBox2D[i].backdownleft[1] - BBox2D[i].backdownright[1]));
        std::cout << "输出后框宽度为：" << dis2 << std::endl;
    }
    // for (int i = 0; i < BBoxes.size(); ++i)
    // {
    //     if (abs(yaws[i]) <= 0.5)
    //     {
    //         yaws[i] = 0;
    //     }
    //     // // frontdownleft
    //     // double d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].frontdownleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].frontdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // backdownleft
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].backdownleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].backdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // frontdownright
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].frontdownright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].frontdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // backdownright
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].backdownright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].backdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // frontdownleft
    //     double d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].frontdownleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].frontdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // backdownleft
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y - mini_dimensionY[i] * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].backdownleft[0] = (int)((BBoxes[i].pose.position.x - mini_dimensionY[i] * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].backdownleft[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // frontdownright
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].frontdownright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].frontdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // backdownright
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y + mini_dimensionY[i] * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].backdownright[0] = (int)((BBoxes[i].pose.position.x + mini_dimensionY[i] * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].backdownright[1] = (int)((BBoxes[i].pose.position.z + BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     double dis1 = sqrt((BBox2D[i].frontdownleft[0] - BBox2D[i].frontdownright[0]) * (BBox2D[i].frontdownleft[0] - BBox2D[i].frontdownright[0]) +
    //                        (BBox2D[i].frontdownleft[1] - BBox2D[i].frontdownright[1]) * (BBox2D[i].frontdownleft[1] - BBox2D[i].frontdownright[1]));
    //     std::cout << "输出前框宽度为：" << dis1 << std::endl;

    //     // // frontupleft
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].frontupleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].frontupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // backupleft
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].backupleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].backupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // frontupright
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].frontupright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].frontupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // // backupright
    //     // d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     // BBox2D[i].backupright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     // BBox2D[i].backupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // backupleft
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y - mini_dimensionY[i] * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].backupleft[0] = (int)((BBoxes[i].pose.position.x - mini_dimensionY[i] * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].backupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);

    //     // frontupleft
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y - BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].frontupleft[0] = (int)((BBoxes[i].pose.position.x - BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].frontupleft[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);
    //     double h = BBox2D[i].frontupleft[1];
    //     // BBox2D[i].backupleft[1] = h - 10;
    //     // double height = BBox2D[i].backdownleft[1] - BBox2D[i].backupleft[1];
    //     // BBox2D[i].frontupleft[1] = h - 5;

    //     // frontupright
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y + BBoxes[i].dimensions.y * sin(yaws[i]) / 2 + BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].frontupright[0] = (int)((BBoxes[i].pose.position.x + BBoxes[i].dimensions.y * cos(yaws[i]) / 2 - BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].frontupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);
    //     // BBox2D[i].frontupright[1] = h - 5;

    //     // backupright
    //     d_tmp = f * b / ((BBoxes[i].pose.position.y + mini_dimensionY[i] * sin(yaws[i]) / 2 - BBoxes[i].dimensions.x * cos(yaws[i]) / 2) * resz);
    //     BBox2D[i].backupright[0] = (int)((BBoxes[i].pose.position.x + mini_dimensionY[i] * cos(yaws[i]) / 2 + BBoxes[i].dimensions.x * sin(yaws[i]) / 2) * d_tmp * resx / b + U0);
    //     BBox2D[i].backupright[1] = (int)((BBoxes[i].pose.position.z - BBoxes[i].dimensions.z / 2) * d_tmp / b + V0);
    //     // BBox2D[i].backupright[1] = h - 10;

    //     double dis2 = sqrt((BBox2D[i].backdownleft[0] - BBox2D[i].backdownright[0]) * (BBox2D[i].backdownleft[0] - BBox2D[i].backdownright[0]) +
    //                        (BBox2D[i].backdownleft[1] - BBox2D[i].backdownright[1]) * (BBox2D[i].backdownleft[1] - BBox2D[i].backdownright[1]));
    //     std::cout << "输出后框宽度为：" << dis2 << std::endl;
    // }
}