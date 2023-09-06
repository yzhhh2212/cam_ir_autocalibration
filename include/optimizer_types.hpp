#ifndef _OPTIMIZER_TYPES_HPP
#define _OPTIMIZER_TYPES_HPP
#include "thirdparty/g2o/g2o/core/base_unary_edge.h"
#include <thirdparty/g2o/g2o/types/types_six_dof_expmap.h>
#include <thirdparty/g2o/g2o/types/sim3.h>

#include <Eigen/Geometry>

#include "camera.hpp"

class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose() {}

    bool read(std::istream &is);

    bool write(std::ostream &os) const;

    void computeError()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);
        Eigen::Vector3d v3D;
        v3D = v1->estimate().map(Xw);
        Eigen::Vector2d res;
        res[0] = camera::fx * v3D[0] / v3D[2] + camera::cx;
        res[1] = camera::fy * v3D[1] / v3D[2] + camera::cy;
        _error = obs - res; // obs:camera ;Xw:ir camera 3d point
    }

    bool isDepthPositive()
    {
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        return (v1->estimate().map(Xw))(2) > 0.0;
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xw;
};


#endif