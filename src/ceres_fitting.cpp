#include "common.h"
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// should be function object
struct CostFunctor{
  CostFunctor(double x, double y):
  _x(x),_y(y)
  {}
  
  template <typename T>
  bool operator()(const T* const abc, T* residual) const{
  // here should be consistent with the dimensions in CostFunction
    residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
    return true;
  }
  
  const double _x,_y;
};

int main(int argc, char** argv){
  
  double a=1.0, b=2.0, c=1.0;
  int N=10000;
  double w_sigma=1.0;
  cv::RNG rng;
  vector<double> x_data, y_data;
//   cout << "generating data:" << endl;
  for(int i=0; i<N; i++){
    double x = i/(double)N;
    x_data.push_back(x);
    y_data.push_back(
      exp(a*x*x+b*x+c) + rng.gaussian(w_sigma)
    );
//     cout << x_data[i] << " " << y_data[i] << endl;
  }
  
  double abc[3];
  ceres::Problem problem;
  
  for(int i=0; i< N; i++){
    ceres::CostFunction* cost_function = 
	  new ceres::AutoDiffCostFunction<CostFunctor, 1, 3>(new CostFunctor(x_data[i], y_data[i]));
    problem.AddResidualBlock(cost_function, nullptr, abc);
  }
  
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_elapse = chrono::duration_cast<chrono::duration<double>>(t2-t1);
  cout << "Time used for optimization = " << time_elapse.count() << " senconds." << endl;
  
  cout << summary.BriefReport() << endl;
  cout << "Estimated a,b,c = ";
  for(auto a:abc) cout << a << " "; 
  cout << endl;
  for(int i=0; i<10; i++) cout << "***" << endl;
  return 0;
}