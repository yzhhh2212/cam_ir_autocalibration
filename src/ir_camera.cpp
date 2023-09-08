#include "ir_camera.hpp"

ircamera::ircamera()
{
    _col = 5;
    _row = 8;
    _square_size = 0.0275;
    // 初始化畸变系数矩阵
    distCoeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    distCoeffs_.at<double>(0, 0) = 0.741852;   // K1
    distCoeffs_.at<double>(0, 1) = -0.585187;  // K2
    distCoeffs_.at<double>(0, 2) = 0.00383753; // P1
    distCoeffs_.at<double>(0, 3) = 0.00138365; // P2
    distCoeffs_.at<double>(0, 4) = -0.239346;  // K3

    // 初始化内参矩阵
    intrinsics_ = cv::Mat::eye(3, 3, CV_64F);
    intrinsics_.at<double>(0, 0) = 467.28;  // Fx
    intrinsics_.at<double>(1, 1) = 467.983; // Fy
    intrinsics_.at<double>(0, 2) = 329.569; // Cx
    intrinsics_.at<double>(1, 2) = 233.152; // Cy
}

bool ircamera::EstimateBoardPose(cv::Mat image)
{
    cv::Size pattern_size(_col, _row);
    cv::Mat gray;
    cv::Mat imgUndistort;
    std::vector<cv::Point2f> corners;
    // 检查图像是否成功读取
    // if (image.empty())
    // {
    //     std::cerr << "Failed to read image from "  << std::endl;
    //     return false; // 或其他适当的错误处理
    // }

    // // 检查内参和畸变系数是否已初始化
    // if (intrinsics_.empty() || distCoeffs_.empty())
    // {
    //     std::cerr << "Intrinsics or distortion coefficients are not initialized." << std::endl;
    //     return false; // 或其他适当的错误处理
    // }
    cv::undistort(image, imgUndistort, intrinsics_, distCoeffs_);
    // cv::cvtColor(imgUndistort, gray, cv::COLOR_BGR2GRAY);
    // imgUndistort = image;

    if (cv::findChessboardCorners(imgUndistort, pattern_size, corners))
    {
        // cv::cornerSubPix(
        //     image, corners, cv::Size(11, 11), cv::Size(-1, -1),
        //     cv::TermCriteria(2 + 1, 30, 0.1));

        cv::cornerSubPix(imgUndistort, corners, cv::Size(11, 11), cv::Size(-1, -1),
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
        }
        else
        {
            std::cout << "Chessboard config is not correct with ir_image\n";
            return false;
        }
    }
    else
    {
        std::cout << "No chessboard detected in ir_image\n";
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

    _Tib.setIdentity();
    _Tib.block<3, 3>(0, 0) = Rcb; // 设置旋转部分
    _Tib.block<3, 1>(0, 3) = tcb; // 设置平移部分

    // std::cout << "The transformation matrix _Tcb is: \n"
    //           << _Tcb << std::endl;
    return true;
}