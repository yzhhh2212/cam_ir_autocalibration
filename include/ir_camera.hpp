#ifndef _IR_CAMERA_HPP
#define _IR_CAMERA_HPP

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

class ircamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ircamera();
    bool EstimateBoardPose(cv::Mat image);
    void ProjectBoard();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectChessboardToCameraSpace();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ChessboardFromCam2TofEstimate();
    bool Project3DTo2D(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    bool UpdatePose(Eigen::Matrix4d Tcl, cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Update3dPose(Eigen::Matrix4d Tcl);

    static double fx;
    static double fy;
    static double cx;
    static double cy;
    std::vector<cv::Point3d> _p3ds;
    std::vector<cv::Point2d> _p2ds;
    Eigen::Matrix4d _Tib;

private:
    // chaseboard para
    int _col;
    int _row;
    float _square_size;
    std::vector<cv::Point3f> _p3ds_plane;

    Eigen::Matrix4d _Tcb;
    Eigen::Matrix4d _Tbc;
    cv::Mat intrinsics_;
    cv::Mat distCoeffs_;
};

#endif