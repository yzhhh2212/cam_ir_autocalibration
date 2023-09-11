#include "optimizer.hpp"

optimizer::optimizer()
{
}

bool optimizer::PoseOptimization(std::vector<std::shared_ptr<camera>> &cameras, std::vector<std::shared_ptr<ircamera>> &ircameras)
{
    // Step 1：构造g2o优化器, BlockSolver_6_3表示：位姿 _PoseDim 为6维，路标点 _LandmarkDim 是3维
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // 输入的帧中,有效的,参与优化过程的2D-3D点对
    int nInitialCorrespondences = 0;

    // Set Frame vertex
    // Step 2：添加顶点：待优化当前帧的Tcw
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
    Eigen::Quaterniond quaternion(camera::_Rci_Original);
    Eigen::Vector3d t = camera::_tci_Original;
    // vSE3->setEstimate(g2o::SE3Quat(quaternion.cast<double>(), camera::_tci_Original.cast<double>()));
    // Eigen::Matrix3d R;
    // R << 0.999711, 0.0223557, 0.00880984,
    //     -0.0223428, 0.999749, -0.00156076,
    //     -0.00884252, 0.00136347, 0.99996;

    // Eigen::Vector3d t(-0.00773844, -0.0782872, 0.0435765);
    Eigen::Quaterniond q(0.999334, 0.00575288, 0.028658, -0.0215528);
    Eigen::Matrix3d R = q.toRotationMatrix();
    // Eigen::Vector3d t(-0.0342119, -0.0685892, 0.0934624);

    g2o::SE3Quat se3quat(quaternion, t);
    vSE3->setEstimate(se3quat);
    // 设置id，保证本次优化过程中id独立即可
    vSE3->setId(0);
    // 要优化的变量，所以不能固定
    vSE3->setFixed(false);
    // 添加节点
    optimizer.addVertex(vSE3);
    const int N = cameras.size() * 8 * 5 + 1;

    std::vector<bool> isbad;
    std::vector<EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono; // 存放单目边
    std::vector<size_t> vnIndexEdgeMono;                  // 边对应特征点的id
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);
    isbad.reserve(N);
    // Set MapPoint vertices

    // vector<optimizeredgese3projectxyzonlypose *> vpedgesmono;             // 存放单目边
    // vector<orb_slam3::edgese3projectxyzonlyposetobody *> vpedgesmono_fhr; // 存放另一目的边
    // vector<size_t> vnindexedgemono, vnindexedgeright;                     // 边对应特征点的id
    // vpedgesmono.reserve(n);
    // vpedgesmono_fhr.reserve(n);
    // vnindexedgemono.reserve(n);
    // vnindexedgeright.reserve(n);

    // vector<g2o::edgestereose3projectxyzonlypose *> vpedgesstereo; // 存放双目边
    // vector<size_t> vnindexedgestereo;
    // vpedgesstereo.reserve(n);
    // vnindexedgestereo.reserve(n);

    // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值5.991
    // 可以理解为卡方值高于5.991 95%的几率维外点
    const float deltaMono = sqrt(5.991);
    // 自由度为3的卡方分布，显著性水平为0.05，对应的临界阈值7.815
    const float deltaStereo = sqrt(7.815);
    long int index = 1;
    for (int i = 0; i < cameras.size(); ++i)
    {
        std::shared_ptr<camera> camera = cameras[i];
        std::shared_ptr<ircamera> ircamera = ircameras[i];

        if (camera->_p2ds.size() != ircamera->_p2ds.size())
        {
            std::cout << "容器点不等，抛弃！！！！！!!!!" << std::endl;
            continue;
        }
        for (int j = 0; j < camera->_p2ds.size(); ++j)
        {
            Eigen::Matrix<double, 2, 1> obs;
            obs << camera->_p2ds[j].x, camera->_p2ds[j].y;
            EdgeSE3ProjectXYZOnlyPose *e = new EdgeSE3ProjectXYZOnlyPose();

            e->setId(index);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e->setMeasurement(obs);

            // const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            // e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
            e->setInformation(Eigen::Matrix2d::Identity());
            // 在这里使用了鲁棒核函数
            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(deltaMono);

            Eigen::Vector4d point_homo(ircamera->_p3ds[j].x, ircamera->_p3ds[j].y, ircamera->_p3ds[j].z, 1.0); // 齐次坐标
            Eigen::Vector4d ir3DP = ircamera->_Tib * point_homo;
            e->Xw << ir3DP(0), ir3DP(1), ir3DP(2);

            optimizer.addEdge(e);
            vpEdgesMono.push_back(e);
            vnIndexEdgeMono.push_back(index);
            isbad.push_back(false);
            index++;
            // vpEdgesMono.push_back(e);
            // vnIndexEdgeMono.push_back(i);
        }
    }
    int nBad = 0;
    int nBad_total = 0;
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};   // 单目
    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815}; // 双目
    const int its[4] = {10, 10, 10, 10};                      // 四次迭代，每次迭代的次数
    for (size_t it = 0; it < 3; it++)
    {
        optimizer.setVerbose(true);
        vSE3->setEstimate(se3quat);
        // 其实就是初始化优化器,这里的参数0就算是不填写,默认也是0,也就是只对level为0的边进行优化
        optimizer.initializeOptimization(0);
        // 开始优化，优化10次
        optimizer.optimize(its[it]);

        nBad = 0;
        // 优化结束,开始遍历参与优化的每一条误差边
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

            bool Isbad = isbad[i];

            // 如果这条误差边是来自于outlier，也就是上一次优化去掉的
            // 重新计算，因为上一次外点这一次可能成为内点
            if (Isbad)
            {
                e->computeError();
            }

            // 就是error*\Omega*error,表征了这个点的误差大小(考虑置信度以后)
            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it])
            {
                // pFrame->mvbOutlier[idx] = true;
                Isbad = true;
                e->setLevel(1); // 设置为outlier , level 1 对应为外点,上面的过程中我们设置其为不优化
                nBad++;
            }
            else
            {
                // pFrame->mvbOutlier[idx] = false;
                Isbad = false;
                e->setLevel(0); // 设置为inlier, level 0 对应为内点,上面的过程中我们就是要优化这些关系
            }

            if (it == 2)
                e->setRobustKernel(0); // 除了前两次优化需要RobustKernel以外, 其余的优化都不需要 -- 因为重投影的误差已经有明显的下降了
        }
        std::cout << "第 " << it + 1 << " 次优化, 本次的外点个数为：" << nBad << std::endl;
        g2o::SE3Quat result = vSE3->estimate();

        camera::_Rci_Optimized = result.rotation().toRotationMatrix();
        camera::_tci_Optimized = result.translation();
        Eigen::Matrix3d R2 = result.rotation().toRotationMatrix();
        Eigen::Vector3d t2 = result.translation();

        Eigen::Quaterniond q2(R2);

        std::cout << " result is " << std::endl;

        // 输出四元数
        std::cout << "rotation: \n";
        std::cout << "w: " << q2.w() << std::endl;
        std::cout << "x: " << q2.x() << std::endl;
        std::cout << "y: " << q2.y() << std::endl;
        std::cout << "z: " << q2.z() << std::endl;

        // 输出平移向量
        std::cout << "translation: \n";
        std::cout << "x: " << t2.x() << std::endl;
        std::cout << "y: " << t2.y() << std::endl;
        std::cout << "z: " << t2.z() << std::endl;
        nBad_total += nBad;
    }
    std::cout << "一共有 " << index << "个点" << std::endl;
    // std::cout << "一共有 " << nBad << "个外点" << std::endl;
    // optimizer.initializeOptimization();
    // optimizer.optimize(50);

    g2o::SE3Quat result = vSE3->estimate();

    camera::_Rci_Optimized = result.rotation().toRotationMatrix();
    camera::_tci_Optimized = result.translation();
    Eigen::Matrix3d R2 = result.rotation().toRotationMatrix();
    Eigen::Vector3d t2 = result.translation();

    Eigen::Quaterniond q2(R2);

    std::cout << "final result is " << std::endl;

    // 输出四元数
    std::cout << "rotation: \n";
    std::cout << "w: " << q2.w() << std::endl;
    std::cout << "x: " << q2.x() << std::endl;
    std::cout << "y: " << q2.y() << std::endl;
    std::cout << "z: " << q2.z() << std::endl;

    // 输出平移向量
    std::cout << "translation: \n";
    std::cout << "x: " << t2.x() << std::endl;
    std::cout << "y: " << t2.y() << std::endl;
    std::cout << "z: " << t2.z() << std::endl;
    return true;
}