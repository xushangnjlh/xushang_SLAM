#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>
#include <sophus/se3.h>

int main()
{
  // construct SO3 in Sophus
  Sophus::SO3 SO3_v(0,0,M_PI/2);
  cout << "From vector: " << SO3_v << endl;

  Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
  Sophus::SO3 SO3_R(R);
  cout << "R = " << endl
       << R << endl
       << "From Matrix R: " << SO3_R << endl;
       
  Eigen::Quaterniond q(R);
  Sophus::SO3 SO3_q(q);
  cout << "q = "
       << q.w() << " " <<q.x() << " " << q.y() << " " << q.z() << endl
       << "From Quaternion q: " << SO3_q << endl;
  
  Eigen::Vector3d so3 = SO3_R.log();
  cout << "so3: " << so3.transpose() << endl;
  cout << "so3 hat " << endl << Sophus::SO3::hat(so3) << endl;
  cout << "so3 hat vee" << endl << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;
  
  Eigen::Vector3d update_so3(1e-4,0,0);
  Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3)*SO3_R;
  cout << "SO3_updated: " << endl << SO3_updated.matrix() << endl;
  
  cout << "******我是分割线******" << endl;
  
  Eigen::Vector3d t(1,0,0);
  Sophus::SE3 SE3_Rt(R,t);
  Sophus::SE3 SE3_qt(q,t);
  cout << "SE3 from R t: " << endl << SE3_Rt << endl;
  cout << "SE3 from q,t: " << endl << SE3_qt << endl;
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  
  cout << "se3 hat " << endl << Sophus::SE3::hat(se3) << endl;
  cout << "se3 hat vee " << endl << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;
  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0,0) = 1e-4;
  Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3)*SE3_Rt;
  cout << "SE3_updated: " << endl << SE3_updated.matrix() << endl;
  
  Eigen::Matrix3d d;
  d << 1,2,3,4,5,6,7,8,9;
  cout << d << endl;
  d.block(1,1,2,2) = Eigen::Matrix<double, 2,2>::Zero();
  cout << d << endl;
  cout <<"*";
  Sophus::SE3 e;
  cout << e << endl;
  return 0;
}