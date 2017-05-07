#include <ceres/ceres.h>
using namespace std;

using ceres::Problem;
using ceres::CostFunction;
using ceres::Solver;
using ceres::Solve;

struct CostFunctor{
  template <typename T>
  bool operator()(const T* const x, T* residual) const{
    residual[0] = T(10.0)-x[0];
    return true;
  }
};

int main(int argc, char** argv){
  double initial_x = 5.0;
  double x = initial_x;
  
  Problem problem;
  CostFunction* costFunction = new ceres::AutoDiffCostFunction<CostFunctor,1,1>(new CostFunctor);
  problem.AddResidualBlock(costFunction, nullptr, &x);
  
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout=true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
  cout << summary.BriefReport() << endl;
  cout << "x = " << initial_x << " -> " << x << endl;
  return 0;
}