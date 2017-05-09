#include "projectionModel.h"
#include <ceres/ceres.h>

class ReprojectionError
{
public:
  ReprojectionError(double observedx, double observedy)
  :observedx_(observedx), observedy_(observedy)
  {}
  
  template <typename T>
  bool operator()(const T* const camera, const T* const point, T* residuals) const
  {
    T predictions[2];
    CamProjectionWithDistortion<T>(camera, point, predictions);
    residuals[0] = observedx_ - predictions[0];
    residuals[1] = observedy_ - predictions[1];
    
    return true;
  }
  
  // Factory to hide the construction of the CostFunction object from
  // the client code.
  
  static ceres::CostFunction* Create(const double observedx, const double observedy)
  {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>
	(new ReprojectionError(observedx, observedy)));
  }
  
private:
  double observedx_;
  double observedy_;
};