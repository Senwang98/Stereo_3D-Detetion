
#ifndef DISPARITY_DISPLAY_COLORCCA_H
#define DISPARITY_DISPLAY_COLORCCA_H

#include <opencv2/opencv.hpp>
#include <gflags/gflags.h>
#include <stack>
#include "objectdetection_buildminbox.h"
#include "objectdetection_dbscan.h"

struct eightPoints2D
{
    cv::Vec2i frontupleft;
    cv::Vec2i frontupright;
    cv::Vec2i frontdownleft;
    cv::Vec2i frontdownright;
    cv::Vec2i backupleft;
    cv::Vec2i backupright;
    cv::Vec2i backdownleft;
    cv::Vec2i backdownright;
    cv::Vec2i center;
};

class colorCCA
{
public:
    /// construct of multi value CCA
    /// \param input_img
    /// \param input_label
    /// \param b_in
    /// \param f_in
    /// \param resx_in
    /// \param resz_in
    colorCCA(cv::Mat &input_img, cv::Mat &input_label,
             double b_in, double f_in, double resx_in, double resz_in, double u0, double v0,
             cv::Mat &src_img);

    /// segment the gray disparity_img using seedfilling
    /// \param border
    /// \return
    bool colorsegment_seedfill(int border);

    /// get bbox of objects in world, and transform to 2D
    void getBbox(int i);
    cv::Vec3b static make_color(int id);

private:
    /// calculate Euclidian distance between two pixels' value
    /// \param r1
    /// \param c1
    /// \param r2
    /// \param c2
    /// \return
    bool dist(int r1, int c1, int r2, int c2);

public:
    cv::Mat *idxImg;              // gray disparity_img 完成seedfill与grid点云生成
    std::vector<int> label_count; // numbers of labels
    std::vector<eightPoints2D> BBox2D;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obj_pcls;
    std::vector<jsk_recognition_msgs::BoundingBox> BBoxes;
    std::vector<double> yaws;

private:
    cv::Mat *disparity_img; // disparity map (gray disparity_img)
    cv::Mat XZobj;          // objs in xz plane 暂未使用
    int threshold_similar;
    double b;    // baseline
    double f;    // focal length
    double resx; // resolution in x
    double resz; // resolution in z
    double U0;
    double V0;
    cv::Mat inner;
    cv::Mat outer;
    std::vector<double> mini_dimensionY;
    cv::Mat *src;
};

#endif //DISPARITY_DISPLAY_COLORCCA_H
