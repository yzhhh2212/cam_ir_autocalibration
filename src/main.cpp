#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "camera.hpp"
#include "ir_camera.hpp"
#include "optimizer.hpp"
#include <memory>
#include <pcl/visualization/cloud_viewer.h>

std::mutex cloud_mutex; // 用于保护点云数据的互斥锁
pcl::PointCloud<pcl::PointXYZ>::Ptr boardcloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

double delta_angle = 0.001;            // 欧拉角调整量
double delta_trans = 0.001;            // 平移调整量
Eigen::Vector3d euler_angles(0, 0, 0); // 欧拉角
Eigen::Vector3d translation(0, 0, 0);  // 平移向量

void VisualizationCallback(pcl::visualization::PCLVisualizer &viz)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    std::unique_lock<std::mutex> lockOnCallBack(cloud_mutex);
    {
        if (!viz.updatePointCloud(cloud, red, "tof_cloud"))
        {
            viz.addPointCloud(cloud, red, "tof_cloud");
        }

        if (!viz.updatePointCloud(boardcloud, green, "board_cloud"))
        {
            viz.addPointCloud(boardcloud, green, "board_cloud");
        }
    }
}

// bool readYaml(Eigen::Matrix3d &Rcl, Eigen::Vector3d &tcl)
// {
//     std::ifstream f("/home/yzhhh/project/keystar/cam_tof_manual/estimate_pose.yaml");
//     if (f.good())
//     {
//         f.close();
//         cv::FileStorage fs("/home/yzhhh/project/keystar/cam_tof_manual/estimate_pose.yaml", cv::FileStorage::READ);
//         // 你的其他代码

//         if (!fs.isOpened())
//         {
//             std::cerr << "Failed to open the YAML file!" << std::endl;
//             return false;
//         }

//         // 读取旋转四元数
//         cv::FileNode rotationNode = fs["rotation"];
//         double w = (double)rotationNode["w"];
//         double x = (double)rotationNode["x"];
//         double y = (double)rotationNode["y"];
//         double z = (double)rotationNode["z"];

//         // 读取平移向量
//         cv::FileNode translationNode = fs["translation"];
//         double tx = (double)translationNode["x"];
//         double ty = (double)translationNode["y"];
//         double tz = (double)translationNode["z"];

//         // 将四元数转换为旋转矩阵（使用 Eigen 库）
//         Eigen::Quaterniond quat(w, x, y, z);
//         Rcl = quat.toRotationMatrix();

//         // 创建平移向量（使用 Eigen 库）
//         tcl << tx, ty, tz;

//         // 输出读取到的值
//         std::cout << "Rotation Matrix: \n"
//                   << Rcl << std::endl;
//         std::cout << "Translation Vector: \n"
//                   << tcl << std::endl;

//         return true;
//     }
//     else
//     {
//         std::cout << "文件不存在或无法打开" << std::endl;
//     }
//     // 创建一个 FileStorage 对象，用于读取 YAML 文件
// }

int main()
{
    std::string folderPath = "/usr/local/project/keystar/rgb_ir_image/"; // 例如 "C:/images/"
    std::string fileExtension = ".jpg";                                       // 图片的扩展名
    std::string irimgExtension = ".jpg";                                      // 图片的扩展名
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.runOnVisualizationThread(VisualizationCallback);
    double w = 0.999334;
    double x = 0.00683661;
    double y = 0.0270457;
    double z = -0.0235022;

    camera::_Rcl_Estimate = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
    camera::_tcl_Estimate[0] = -0.0227738;
    camera::_tcl_Estimate[1] = -0.0660415;
    camera::_tcl_Estimate[2] = 0.0939607;
    camera::_Tcl_Estimate.setIdentity();
    camera::_Tcl_Estimate.block<3, 3>(0, 0) = camera::_Rcl_Estimate; // 设置旋转部分
    camera::_Tcl_Estimate.block<3, 1>(0, 3) = camera::_tcl_Estimate; // 设置平移部分
    camera::_Tlc_Estimate = camera::_Tcl_Estimate.inverse();
    std::vector<std::shared_ptr<camera>> cameras;
    std::vector<std::shared_ptr<ircamera>> ircameras;

    for (int i = 1; i <= 319 ;++i)
    {
        std::shared_ptr<camera> rgb_camera(new camera());
        std::shared_ptr<ircamera> ir_camera(new ircamera());
        std::stringstream ss;
        std::stringstream ssir;
        ss << folderPath << "rgb_" << i << fileExtension; // 拼接完整的文件路径
        ssir << folderPath << "ir_" << i << irimgExtension;
        std::string filePath = ss.str();
        std::string irimgPath = ssir.str();

        cv::Mat gray;

        cv::Mat image = cv::imread(filePath, cv::IMREAD_COLOR); // 读取图片
        // cv::Mat raw_image = cv::imread(filePath, cv::IMREAD_COLOR); // 读取图片
        if (image.empty())
        {
            std::cerr << "Failed to read image from " << filePath << std::endl;
            continue; // Skip the current iteration
        }

        cv::Mat irimage = cv::imread(irimgPath, cv::IMREAD_GRAYSCALE); // 读取图片
        if (irimage.empty())
        {
            std::cerr << "Failed to read image from " << irimgPath << std::endl;
            continue; // Skip the current iteration
        }
        // cv::imshow("img",image);
        // cv::imshow("irimg",irimage);
        // cv::waitKey(1);

        // cv::Mat ir_raw_image = cv::imread(irimgPath, cv::IMREAD_GRAYSCALE); // 读取图片
        // cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        if (!rgb_camera->EstimateBoardPose(image) || !ir_camera->EstimateBoardPose(irimage))
        {
            std::cout << "图片角点不匹配，抛弃图片 " << i << std::endl;
            continue;
        }
        cameras.push_back(rgb_camera);
        ircameras.push_back(ir_camera);
    }
    // camera::_Tci_Original = cameras[0]->_Tcb * ircameras[0]->_Tib.inverse();
    // camera::_Rci_Original = camera::_Tci_Original.block<3, 3>(0, 0);
    // camera::_tci_Original = camera::_Tci_Original.block<3, 1>(0, 3);
    std::shared_ptr<optimizer> pose_optimizer(new optimizer());
    std::cout<<"size of cameras " << cameras.size() << std::endl;
    std::cout<<"size of ircameras " << ircameras.size() << std::endl;
    Eigen::Matrix4d initial_Tci =  pose_optimizer->ComputeInitialT(cameras,ircameras);
    if (pose_optimizer->PoseOptimization(cameras, ircameras,initial_Tci))
    {
        std::cout << "optimize success" << std::endl;
    }
}
