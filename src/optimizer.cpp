#include "optimizer.hpp"

bool optimizer::PoseOptimization(std::vector<camera *> cameras, std::vector<ircamera *> ircameras)
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
    vSE3->setEstimate(g2o::SE3Quat(quaternion, camera::_tci_Original));

    // 设置id，保证本次优化过程中id独立即可
    vSE3->setId(0);
    // 要优化的变量，所以不能固定
    vSE3->setFixed(false);
    // 添加节点
    optimizer.addVertex(vSE3);

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
        camera *camera = cameras[i];
        ircamera *ircamera = ircameras[i];

        if (camera->_p2ds.size() != ircamera->_p2ds.size())
            continue;
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

            Eigen::Vector4d point_homo(ircamera->_p3ds[j].x, ircamera->_p3ds[j].y, ircamera->_p3ds[j].z, 0.0); // 齐次坐标
            Eigen::Vector4d ir3DP = ircamera->_Tib * point_homo;
            e->Xw << ir3DP(0), ir3DP(1), ir3DP(2);

            optimizer.addEdge(e);
            index++;
            // vpEdgesMono.push_back(e);
            // vnIndexEdgeMono.push_back(i);
        }
    }
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::SE3Quat result = vSE3->estimate();

    camera::_Rci_Optimized = result.rotation().toRotationMatrix();
    camera::_tci_Optimized = result.translation();

    camera::_Tci_Optimized.block<3, 3>(0, 0) = camera::_Rci_Optimized;
    camera::_Tci_Optimized.block<3, 1>(0, 3) = camera::_tci_Optimized;
}