#include <iostream>
#include <fstream>
#include "ceres/ceres.h"

#include "ReprojectionError.h"
#include "BALProblem.h"
#include "BundleParams.h"

using namespace std;
using namespace ceres;

// add each residual blocks to 
// objective function:
//      1  --m--   --n--				        2
// min ---  >	    >	||observations(i,j) - h(camera, point)||
//	2  --1--   --1--

void BuildProblem(BALProblem* bal_problem, Problem* problem, const BundleParams& params)
{
  const int point_block_size = bal_problem->point_block_size();
  const int camera_block_size = bal_problem->camera_block_size();
  //TODO
//   double* points = bal_problem->mutable_points();
//   double* camera = bal_problem->mutable_cameras();
  const double* observations = bal_problem->observations();
  
  for(int i=0; i<bal_problem->num_observations(); i++)
  {
    CostFunction* costFunction = ReprojectionError::Create(observations[2*i],observations[2*i+1]);
    LossFunction* lossFunction = params.robustify ? new HuberLoss(1.0) : nullptr;
    problem->AddResidualBlock(costFunction, 
			      lossFunction, 
			      bal_problem->mutable_camera_for_observation(i), 
			      bal_problem->mutable_point_for_observation(i)
 			    );
  }
}

// marginalize points first
void SetOrdering(BALProblem* bal_problem, ceres::Solver::Options* options, const BundleParams& params)
{
  if(params.ordering == "automatic") return;

  ceres::ParameterBlockOrdering* ordering = new ceres::ParameterBlockOrdering;

  for(int i=0; i<bal_problem->num_points(); i++)
  {
    ordering->AddElementToGroup(bal_problem->mutable_point_for_observation(i),0);
  }

  for(int i=0; i<bal_problem->num_cameras(); i++)
  {
    ordering->AddElementToGroup(bal_problem->mutable_camera_for_observation(i),1);
  }
  
  options->linear_solver_ordering.reset(ordering);
}

// set other options
void SetSolverOptionsFromParams(BALProblem* bal_problem, ceres::Solver::Options* options, const BundleParams& params)
{
  options->max_num_iterations = params.num_iterations;
  // For Jacobian
  options->num_threads = params.num_threads;
  // For linear solver
  options->num_linear_solver_threads = params.num_threads;
  options->minimizer_progress_to_stdout = true;
  
  CHECK(StringToTrustRegionStrategyType(params.trust_region_strategy, &options->trust_region_strategy_type));

  CHECK(StringToLinearSolverType(params.linear_solver, &options->linear_solver_type));
  CHECK(StringToDenseLinearAlgebraLibraryType(params.dense_linear_algebra_library, &options->dense_linear_algebra_library_type));
  CHECK(StringToSparseLinearAlgebraLibraryType(params.sparse_linear_algebra_library, &options->sparse_linear_algebra_library_type));
  
  SetOrdering(bal_problem, options, params);
}

void SolveProblem(const char* filename, const BundleParams& params)
{
    BALProblem bal_problem(filename);

    // show some information here ...
    std::cout << "bal problem file loaded..." << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points. " << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observatoins. " << std::endl;

    // store the initial 3D cloud points and camera pose..
    if(!params.initial_ply.empty()){
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    std::cout << "beginning problem..." << std::endl;
    
    // add some noise for the intial value
    srand(params.random_seed);
    bal_problem.Normalize();
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma,
                        params.point_sigma);

    std::cout << "Normalization complete..." << std::endl;
    
    Problem problem;
    BuildProblem(&bal_problem, &problem, params);

    std::cout << "the problem is successfully build.." << std::endl;
   
   
    Solver::Options options;
    SetSolverOptionsFromParams(&bal_problem, &options, params);
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // write the result into a .ply file.   
    if(!params.final_ply.empty()){
        bal_problem.WriteToPLYFile(params.final_ply);  // pay attention to this: ceres doesn't copy the value into optimizer, but implement on raw data! 
    }
}

int main(int argc, char** argv)
{    
    BundleParams params(argc,argv);  // set the parameters here.
   
    google::InitGoogleLogging(argv[0]);
    std::cout << params.input << std::endl;
    if(params.input.empty()){
        std::cout << "Usage: ceres_bal -input <path for dataset>" << endl;
        return 1;
    }

    SolveProblem(params.input.c_str(), params);
 
    return 0;
}






























