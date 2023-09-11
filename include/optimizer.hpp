#ifndef _OPTIMIZER_HPP
#define _OPTIMIZER_HPP
// 以相机像素坐标为观测值
#include "optimizer_types.hpp"
#include "ir_camera.hpp"

#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "thirdparty/g2o/g2o/core/block_solver.h"
#include "thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

class optimizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    optimizer();

    bool PoseOptimization(std::vector<std::shared_ptr<camera>> &cameras, std::vector<std::shared_ptr<ircamera>> &ircameras);
};

#endif