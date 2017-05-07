/*
 * Robust Curve Fitting for y = exp(mx+c)
 * 
 * Use OpenCV RNG to generate data
 */

#include "common.h"
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CostFunctor{
  CostFunctor(double x, double y):
  _x(x),_y(y)
  {}
  
  template <typename T>
  bool operator()(const T* const m, const T* const c, T* residual) const{
  // here should be consistent with the dimensions in CostFunction
    residual[0] = T(_y) - ceres::exp( m[0]*T(_x) + c[0] );
    return true;
  }
  
private:
  const double _x,_y;
};

int main(int argc, char** argv){
  
  double m_=1.0, c_=2.0;
  int N=100;
  double w_sigma=2.0;
  cv::RNG rng;
  vector<double> x_data, y_data;
  for(int i=0; i<N; i++){
    double x = i/(double)N;
    x_data.push_back(x);
    if(i==1)
      y_data.push_back(1);
    else
      y_data.push_back(
	exp(m_*x+c_) + rng.gaussian(w_sigma) );
  }
  
  double totol_cost;
  for(int i=0; i<N; i++){
    if(i%9!=1)
    totol_cost += y_data[i] - exp( 1.0*x_data[i]+2 );
  }
  cout << "Final cost of true data: " << totol_cost << endl;;
  
  double m=0, c=0;
  ceres::Problem problem;
  
  for(int i=0; i< N; i++){
    ceres::CostFunction* cost_function = 
	  new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1>(new CostFunctor(x_data[i], y_data[i]));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(2), &m, &c);
  }
  
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  cout << summary.BriefReport() << endl;
  cout << "Estimated m = " << m << ", c = " << c << endl;
  // for clean format
  for(int i=0; i<10; i++) cout << "***" << endl;
  return 0;
}