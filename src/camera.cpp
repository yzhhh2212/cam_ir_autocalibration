#include "camera.hpp"

Eigen::Matrix4d camera::_Tcl_Estimate = Eigen::Matrix4d::Identity();
Eigen::Matrix4d camera::_Tlc_Estimate = Eigen::Matrix4d::Identity();
Eigen::Matrix3d camera::_Rcl_Estimate = Eigen::Matrix3d::Identity();
Eigen::Vector3d camera::_tcl_Estimate = Eigen::Vector3d::Zero();

Eigen::Matrix4d camera::_Tcl_manual = Eigen::Matrix4d::Identity();
Eigen::Matrix4d camera::_Tlc_manual = Eigen::Matrix4d::Identity();
Eigen::Matrix3d camera::_Rcl_manual = Eigen::Matrix3d::Identity();
Eigen::Vector3d camera::_tcl_manual = Eigen::Vector3d::Zero();

Eigen::Matrix4d camera::_Tci_Optimized = Eigen::Matrix4d::Identity();
Eigen::Matrix3d camera::_Rci_Optimized = Eigen::Matrix3d::Identity();
Eigen::Vector3d camera::_tci_Optimized = Eigen::Vector3d::Zero();
Eigen::Matrix4d camera::_Tci_Original = Eigen::Matrix4d::Identity();
Eigen::Matrix3d camera::_Rci_Original = Eigen::Matrix3d::Identity();
Eigen::Vector3d camera::_tci_Original = Eigen::Vector3d::Zero();

double camera::fx = 700.225972;
double camera::fy = 681.416116;
double camera::cx = 631.705476;
double camera::cy = 346.033095;

camera::camera()
{
    _col = 5;
    _row = 8;
    _square_size = 0.0275;
    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    distCoeffs_.at<double>(0, 0) = -0.311801;
    distCoeffs_.at<double>(0, 1) = 0.073565;
    distCoeffs_.at<double>(0, 2) = 0.002223;
    distCoeffs_.at<double>(0, 3) = 0.001517;
    distCoeffs_.at<double>(0, 4) = 0;

    intrinsics_ = cv::Mat::eye(3, 3, CV_64F);
    intrinsics_.at<double>(0, 0) = 700.225972;
    intrinsics_.at<double>(1, 1) = 681.416116;
    intrinsics_.at<double>(0, 2) = 631.705476;
    intrinsics_.at<double>(1, 2) = 346.033095;
}

bool camera::EstimateBoardPose(cv::Mat image)
{
    cv::Size pattern_size(_col, _row);
    cv::Mat gray;
    cv::Mat imgUndistort;
    std::vector<cv::Point2f> corners;
    cv::undistort(image, imgUndistort, intrinsics_, distCoeffs_);
    // imgUndistort =image;
    cv::cvtColor(imgUndistort, gray, cv::COLOR_BGR2GRAY);

    if (cv::findChessboardCorners(imgUndistort, pattern_size, corners))
    {
        // cv::cornerSubPix(
        //     image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        //     cv::TermCriteria(2 + 1, 30, 0.1));
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        if (corners.size() == _col * _row)
        {
            int count = 0;
            for (int i = 0; i < _row; i++)
            {
                for (int j = 0; j < _col; j++)
                {
                    float x = j * _square_size;
                    float y = i * _square_size;
                    // Todo: change 3d-coordinate
                    _p3ds.emplace_back(
                        cv::Point3f(x, y, 0.0));
                    _p2ds.emplace_back(cv::Point2f(corners[count].x, corners[count].y));
                    count++;
                }
            }
            cv::drawChessboardCorners(imgUndistort, pattern_size, cv::Mat(corners), true);
            cv::imshow("image",imgUndistort);
        }
        else
        {
            std::cout << "Chessboard config is not correct with image\n";
            return false;
        }
    }
    else
    {
        std::cout << "No chessboard detected in image\n";
        return false;
    }

    if (_p3ds.size() != _p2ds.size() || _p3ds.size() < 4)
        return false;
    // use PnP to calculate the pose
    cv::Mat r, t;
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);
    cv::solvePnP(_p3ds, _p2ds, intrinsics_, distCoeffs, r, t);
    // std::cout << "solvePnP" << std::endl;
    // type transform
    cv::Mat cvR;
    cv::Rodrigues(r, cvR);

    Eigen::Matrix3d Rcb;
    cv::cv2eigen(cvR, Rcb);
    Eigen::Vector3d tcb;
    cv::cv2eigen(t, tcb);

    _Tcb.setIdentity();
    _Tcb.block<3, 3>(0, 0) = Rcb; // 设置旋转部分
    _Tcb.block<3, 1>(0, 3) = tcb; // 设置平移部分

    // transform matrix
    _Tbc.setIdentity();
    _Tbc.block<3, 3>(0, 0) = Rcb.inverse();
    _Tbc.block<3, 1>(0, 3) = -Rcb.inverse() * tcb;

    // std::cout << "The transformation matrix _Tcb is: \n"
    //           << _Tcb << std::endl;
    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr camera::ProjectChessboardToCameraSpace()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> transformed_p3ds;
    for (const auto &point : _p3ds_plane)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = _Tcb * point_homo;
        // transformed_p3ds.emplace_back(
        //     cv::Point3f(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
        cloud->push_back(pcl::PointXYZ(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
    }
    return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr camera::ChessboardFromCam2TofEstimate()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> transformed_p3ds;
    for (const auto &point : _p3ds_plane)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = _Tlc_Estimate * _Tcb * point_homo;
        // transformed_p3ds.emplace_back(
        //     cv::Point3f(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
        cloud->push_back(pcl::PointXYZ(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
    }
    return cloud;
}

bool camera::Project3DTo2D(cv::Mat &image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (const auto &point : cloud->points)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = _Tcl_Estimate * point_homo;
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2));

        cv::Mat point2D = 1 / point3D.at<double>(2, 0) * intrinsics_ * point3D;

        cv::Point2f pt2D(point2D.at<double>(0, 0), point2D.at<double>(1, 0));
        // 畸变矫正（如果需要）
        // std::vector<cv::Point2f> srcPoints = {cv::Point2f(point2D.at<double>(0, 0), point2D.at<double>(1, 0))};
        // std::vector<cv::Point2f> dstPoints;
        // cv::undistortPoints(srcPoints, dstPoints, intrinsics, distCoeffs);

        // 绘制点
        cv::circle(image, pt2D, 1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
    }
}

bool camera::UpdatePose(Eigen::Matrix4d Tcl, cv::Mat image, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    for (const auto &point : cloud->points)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = Tcl * point_homo;
        cv::Mat point3D = (cv::Mat_<double>(3, 1) << transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2));

        cv::Mat point2D = 1 / point3D.at<double>(2, 0) * intrinsics_ * point3D;

        cv::Point2f pt2D(point2D.at<double>(0, 0), point2D.at<double>(1, 0));

        cv::circle(image, pt2D, 1, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        cv::imshow("image", image);
    }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr camera::Update3dPose(Eigen::Matrix4d Tcl)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point3f> transformed_p3ds;
    for (const auto &point : _p3ds_plane)
    {
        Eigen::Vector4d point_homo(point.x, point.y, point.z, 1.0); // 齐次坐标
        Eigen::Vector4d transformed_point_homo = Tcl.inverse() * _Tcb * point_homo;
        // transformed_p3ds.emplace_back(
        //     cv::Point3f(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
        cloud->push_back(pcl::PointXYZ(transformed_point_homo(0), transformed_point_homo(1), transformed_point_homo(2)));
    }
    return cloud;
}
