#ifndef MODERN_ROBOTICS_H
#define MODERN_ROBOTICS_H

#include <vector>
#include <Eigen/Dense>
//#include <Eigen/Geometry>

namespace MR
{
  struct AxisWAngle;
  bool NearZero(double);
  Eigen::Matrix3d VecToso3(Eigen::Vector3d);
  Eigen::Vector3d so3ToVec(Eigen::Matrix3d);
  AxisWAngle AxisAng3(const Eigen::Vector3d);
  Eigen::Matrix3d MatrixExp3(Eigen::Matrix3d);
  Eigen::Matrix3d MatrixLog3(Eigen::Matrix3d);
  Eigen::Transform<Eigen::ArrayXf::Scalar, 3,  Eigen::TransformTraits::Affine>
  RpToTrans(Eigen::Matrix3d,Eigen::Vector3d);
}
#endif

