#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace std;
using Sophus::SE3;
using Sophus::SO3;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

Matrix6d JRInv(SE3 e)
{
  Matrix6d J;
  J.block(0,0,3,3) = SO3::hat(e.so3().log());
  J.block(0,3,3,3) = SO3::hat(e.translation());
  J.block(3,0,3,3) = Eigen::Matrix3d::Zero(3,3);
  J.block(3,3,3,3) = SO3::hat(e.so3().log());
  return J;
}

typedef Eigen::Matrix<double, 6, 1> Vector6d;
class VertexSE3LieAlgebra:public g2o::BaseVertex<6, SE3>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool read(istream& is)
  {
    double data[7];
    for(int i=0; i<7; i++) is >> data[i];
    setEstimate(SE3(
		    Eigen::Quaterniond(data[6],data[3],data[4],data[5]), // q
		    Eigen::Vector3d(data[0], data[1],data[2]) // t
		   )
	       );
  }
  
  bool write(ostream& os) const
  {
    os<< id() << " ";
    // pay attention to Eigen::Quaterniond, the order of arguments are
    // w,x,y,z in constructor
    // while internally stored like x,y,z,w
    Eigen::Quaterniond q = _estimate.unit_quaternion();
    os << _estimate.translation().transpose() << " ";
    os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
    return true;
  }
  
  virtual void setToOriginImpl()
  {
    _estimate = Sophus::SE3(); // it is SE3, but represent in se3 : 0,0,0,0,0,0
  }
  virtual void oplusImpl(const double* v)
  {
    Sophus::SE3 update(
      Sophus::SO3(v[3],v[4],v[5]),
      Eigen::Vector3d(v[0],v[1],v[2])
    );
    _estimate = update*_estimate;
  }
};

int main()
{
  
}
