#include "mytest/dbscan.h"

double point::getDis(const point &ot)
{
    return sqrt((x - ot.x) * (x - ot.x) + (z - ot.z) * (z - ot.z));
}

// dbsacn聚类
DBSCAN::DBSCAN(int n, double eps, int minPts)
{
    this->n = n;
    this->eps = eps;
    this->minPts = minPts;
    this->clusterIdx = -1;
}

void DBSCAN::run(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
    // init
    // 点云转成point自定义结构体
    this->points.clear();
    for (unsigned int i = 0; i < pointcloud->size(); i++)
    {
        point tmp;
        tmp.x = pointcloud->points[i].x;
        tmp.z = pointcloud->points[i].y;
        tmp.y = pointcloud->points[i].z;
        tmp.ptsCnt = 0;
        tmp.cluster = NOT_CLASSIFIED;
        this->points.push_back(tmp);
    }
    this->size = (int)this->points.size();
    this->adjpoints.resize(this->size);

    // 统计每个点的相邻情况
    checkNearpoints();
    // cout << "相邻点处理成功！" << endl;

    for (int i = 0; i < this->size; i++)
    {
        if (points[i].cluster != NOT_CLASSIFIED)
            continue;

        if (isCoreObject(i))
            dfs(i, ++clusterIdx);
        else
            points[i].cluster = NOISE;
    }

    pointcloud->clear();
    cluster.resize(clusterIdx + 1);
    int cnt_right = 0;
    for (int i = 0; i < this->size; i++)
    {
        if (points[i].cluster != NOISE)
        {
            pcl::PointXYZ tmp;
            tmp.x = points[i].x;
            tmp.y = points[i].z;
            tmp.z = points[i].y;
            pointcloud->push_back(tmp);
            cluster[points[i].cluster].push_back(points[i]);
            cnt_right++;
        }
    }
    cout << "有效点个数伪=" << cnt_right << endl;
    // cout << "DBSCAN处理结束！" << endl;
}

void DBSCAN::dfs(int now, int c)
{
    this->points[now].cluster = c;
    if (!isCoreObject(now))
        return;
    for (auto &next : adjpoints[now])
    {
        if (points[next].cluster != NOT_CLASSIFIED)
            continue;
        dfs(next, c);
    }
}

// 预处理，得到每个点的密度并添加相关点
void DBSCAN::checkNearpoints()
{
    for (int i = 0; i < this->size; i++)
    {
        for (int j = 0; j < this->size; j++)
        {
            if (i == j)
                continue;
            if (points[i].getDis(points[j]) <= eps)
            {
                this->points[i].ptsCnt++;
                this->adjpoints[i].push_back(j);
            }
        }
    }
}

//　是否是核心点
bool DBSCAN::isCoreObject(int idx)
{
    return this->points[idx].ptsCnt >= minPts;
}

vector<vector<point>> DBSCAN::get_cluster()
{
    vector<vector<point>> ans;
    for (int i = 0; i < ans.size(); i++)
        ans.clear();
    for (int i = 0; i < this->cluster.size(); i++)
    {
        if (this->cluster[i].size() <= 50)
            continue;
        vector<point> tmp_ans;
        for (int j = 0; j < this->cluster[i].size(); j++)
        {
            tmp_ans.push_back(this->cluster[i][j]);
        }
        ans.push_back(tmp_ans);
    }
    return ans;
}
