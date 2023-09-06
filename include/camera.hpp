#ifndef _CAMERA_H_
#define _CAMERA_H_
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <Eigen/Core>

class camera
{
public:
    camera();
    bool EstimateBoardPose(cv::Mat image);
    void ProjectBoard();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectChessboardToCameraSpace();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ChessboardFromCam2TofEstimate();
    bool Project3DTo2D(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    bool UpdatePose(Eigen::Matrix4d Tcl, cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Update3dPose(Eigen::Matrix4d Tcl);
    static Eigen::Matrix4d _Tcl_Estimate; // estimate laser to camera
    static Eigen::Matrix3d _Rcl_Estimate;
    static Eigen::Vector3d _tcl_Estimate;
    static Eigen::Matrix4d _Tlc_Estimate;

    static Eigen::Matrix4d _Tcl_manual; // estimate laser to camera
    static Eigen::Matrix3d _Rcl_manual;
    static Eigen::Vector3d _tcl_manual;
    static Eigen::Matrix4d _Tlc_manual;
    static double fx;
    static double fy;
    static double cx;
    static double cy;

    static Eigen::Matrix4d _Tci_Original; // ir 2 cam pose before optimize
    static Eigen::Matrix3d _Rci_Original;
    static Eigen::Vector3d _tci_Original;
    static Eigen::Matrix4d _Tci_Optimized; // ir 2 cam pose after optimize
    static Eigen::Matrix3d _Rci_Optimized;
    static Eigen::Vector3d _tci_Optimized;
    std::vector<cv::Point3f> _p3ds;
    std::vector<cv::Point2f> _p2ds;
    Eigen::Matrix4d _Tcb;

private:
    // chaseboard para
    int _col;
    int _row;
    float _square_size;
    std::vector<cv::Point3f> _p3ds_plane;

    Eigen::Matrix4d _Tbc;
    cv::Mat intrinsics_;
    cv::Mat distCoeffs_;
};

#endif
