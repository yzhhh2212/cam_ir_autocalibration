#include "optimizer_types.hpp"

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream &is)
{
    for (int i = 0; i < 2; i++)
    {
        is >> _measurement[i];
    }
    for (int i = 0; i < 2; i++)
        for (int j = i; j < 2; j++)
        {
            is >> information()(i, j);
            if (i != j)
                information()(j, i) = information()(i, j);
        }
    return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream &os) const
{

    for (int i = 0; i < 2; i++)
    {
        os << measurement()[i] << " ";
    }

    for (int i = 0; i < 2; i++)
        for (int j = i; j < 2; j++)
        {
            os << " " << information()(i, j);
        }
    return os.good();
}

void EdgeSE3ProjectXYZOnlyPose::linearizeOplus()
{
    g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;
    // Eigen::Matrix<double, 3, 6> SE3deriv;
    // SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
    //     -z, 0.f, x, 0.f, 1.f, 0.f,
    //     y, -x, 0.f, 0.f, 0.f, 1.f;

    // _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * SE3deriv;
    _jacobianOplusXi(0,0) = x * y / z_2 * camera::fx;
    _jacobianOplusXi(0,1) = -(1 + (x*x / z_2)) * camera::fx;
    _jacobianOplusXi(0,2) = y / z * camera::fx;
    _jacobianOplusXi(0,3) = -1 / z * camera::fx; 
    _jacobianOplusXi(0,4) = 0;
    _jacobianOplusXi(0,5) = x / z_2 * camera::fx;
    _jacobianOplusXi(1,0) = (1 + y * y / z_2) * camera::fy;
    _jacobianOplusXi(1,1) = -x * y / z_2 * camera::fy;
    _jacobianOplusXi(1,2) = -x / z * camera::fy;
    _jacobianOplusXi(1,3) = 0;
    _jacobianOplusXi(1,4) = -1 / z * camera::fy;
    _jacobianOplusXi(1,5) = y / z_2 * camera::fy;

}
