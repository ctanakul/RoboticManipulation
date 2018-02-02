#ifndef MODERN_ROBOTICS_H
#define MODERN_ROBOTICS_H

#include <vector>
#include <Eigen/Dense>

namespace MR
{
  struct AxisWAngle;
  bool NearZero(double);
  Eigen::Matrix3d VecToso3(Eigen::Vector3d);
  Eigen::Vector3d so3ToVec(Eigen::Matrix3d);
  AxisWAngle AxisAng3(Eigen::Vector3d);
}
#endif

