/*
***************************************************************************
Library of functions written to accompany the algorithms described in
Modern Robotics: Mechanics, Planning, and Control.
***************************************************************************
Adapted from the Python version
Translator: Chainatee Tanakulrungson 
Date: January 2018
***************************************************************************
Language: C++
Included libraries: iostream, math.h, Eigen
***************************************************************************
*/

#include "modern_robotics.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <sstream>


/*
  BASIC HELPER FUNCTIONS
*/

bool MR::NearZero(double z)
{
  /*
    Takes a scalar.
    Checks if the scalar is small enough to be neglected.
    Example Input:
    z = -1e-7
    Output:
    True
   */
  return fabs(z) < 1e-6;
}

/*
  CHAPTER 3: RIGID-BODY MOTIONS
*/

struct MR::AxisWAngle
{
  Eigen::Vector3d axis;
  double angle;
  // first constructor
  AxisWAngle(Eigen::Vector3d omega, double theta)
    try
      : axis(omega), angle(theta) 
      {
	if(fabs(axis.norm() - 1) > 1e-6)
	  {
	    std::stringstream err;
	    err << "Omega axis is not a unit vector: norm = " << axis.norm() << " not 1";
	    throw std::invalid_argument(err.str());
	  }
      }
  catch(const std::invalid_argument& ia)
    {
      std::cout << "Exception: " << ia.what() << std::endl;
    }
  // second constructor
  AxisWAngle(double omg0, double omg1, double omg2, double theta)
    try
      : axis(omg0, omg1, omg2), angle(theta) 
      {
	if(fabs(axis.norm() - 1) > 1e-6)
	  {
	    std::stringstream err;
	    err << "Omega axis is not a unit vector: norm = " << axis.norm() << " not 1";
	    throw std::invalid_argument(err.str());
	  }
      }
  catch(const std::invalid_argument& ia)
    {
      std::cout << "Exception: " << ia.what() << std::endl;
    }
};

const Eigen::Matrix3d EYE3D = Eigen::Matrix3d::Identity();
const Eigen::Matrix3d ZERO3D = Eigen::Matrix3d::Zero();
const double PI = 3.1415926535897;

Eigen::Matrix3d MR::VecToso3(Eigen::Vector3d omg)
{
  /*
    Takes a 3-vector (angular velocity).
    Returns the skew symmetric matrix in so3.
    Example Input: 
    omg = [1, 2, 3]
    Output:
    [[ 0, -3,  2],
    [ 3,  0, -1],
    [-2,  1,  0]]
  */
  Eigen::Matrix3d mat;
  mat <<
    0, -omg(2), omg(1),
    omg(2), 0, -omg(0),
    -omg(1), omg(0), 0;
  return mat;
 
}

Eigen::Vector3d MR::so3ToVec(Eigen::Matrix3d so3mat)
{
  /*
    Takes a 3x3 skew-symmetric matrix (an element of so(3)).
    Returns the corresponding vector (angular velocity).
    Example Input: 
    so3mat = [[ 0, -3,  2],
    [ 3,  0, -1],
    [-2,  1,  0]]
    Output:
    [1, 2, 3]
  */
  Eigen::Vector3d v;
  v <<
    so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
  return v;
  
}

MR::AxisWAngle MR::AxisAng3(const Eigen::Vector3d expc3)
{
  /*
    Takes A 3-vector of exponential coordinates for rotation.
    Returns unit rotation axis omghat and the corresponding rotation angle
    theta.
    Example Input: 
    expc3 = [1, 2, 3]
    Output:
    ([0.2672612419124244, 0.5345224838248488, 0.8017837257372732],
    3.7416573867739413) 
   */
  Eigen::Vector3d ax = expc3;
  ax.normalize();
  //ax = expc3.normalize();
  double ang = expc3.norm();
  MR::AxisWAngle awa(ax, ang);
  //awa.axis = expc3.normalize();
  return awa;
}

Eigen::Matrix3d MR::MatrixExp3(Eigen::Matrix3d so3mat)
{
  /*
    Takes a so(3) representation of exponential coordinates.
    Returns R in SO(3) that is achieved by rotating about omghat by theta from
    an initial orientation R = I.
    Example Input: 
    so3mat = [[ 0, -3,  2],
    [ 3,  0, -1],
    [-2,  1,  0]]
    Output:
    [[-0.69492056,  0.71352099,  0.08929286],
    [-0.19200697, -0.30378504,  0.93319235],
    [ 0.69297817,  0.6313497 ,  0.34810748]]
 */
  Eigen::Vector3d omgtheta = MR::so3ToVec(so3mat);  
  if (NearZero(omgtheta.norm()))
    {
      return EYE3D;
    }
  else
    {
      MR::AxisWAngle omgthetasplit = MR::AxisAng3(omgtheta);
      double theta = omgthetasplit.angle; //3.74166
      Eigen::Matrix3d omgmat = so3mat / theta;
      return EYE3D + (sin(theta) * omgmat) + (1 - cos(theta)) * (omgmat*omgmat);
    }
}

Eigen::Matrix3d MR::MatrixLog3(const Eigen::Matrix3d R)
{
  /*
    Takes R (rotation matrix).
    Returns the corresponding so(3) representation of exponential coordinates.
    Example Input: 
    R = [[0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]]
    Output:
    [[          0, -1.20919958,  1.20919958],
    [ 1.20919958,           0, -1.20919958],
    [-1.20919958,  1.20919958,           0]]
   */
  if (NearZero((R - EYE3D).norm()))
    {
      return ZERO3D;
    }
  else if (NearZero(R.trace() + 1))
    {
      Eigen::Vector3d V;
      if (!(NearZero(1 + R(2,2))))
	{
	  V << R(0,2), R(1,2), 1 + R(2,2);
	  V *= 1.0 / sqrt(2 * (1 + R(2,2)));
	}
      else if (!(NearZero(1 + R(1,1))))
	{
	  V << R(0,1), 1 + R(1,1), R(2,1);
	  V *= 1.0 / sqrt(2 * (1 + R(1,1)));
	}
      else
	{
	  V << 1 + R(0,0), R(1,0), R(2,0);
	  V *= 1.0 / sqrt(2 * (1 + R(0,0)));
	}
      return MR::VecToso3(V * PI);
    }
  else
    {
      double acosinput = (R.trace() - 1) / 2.0;
      if (acosinput > 1) acosinput = 1;
      else if (acosinput < -1) acosinput = -1;
      double theta = acos(acosinput);
      return (theta / 2.0 / sin(theta)) * (R - R.transpose()); 
    }
}

int main()
{
  Eigen::Matrix3d R;
  R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  std::cout << R << std::endl;
  R = MR::MatrixLog3(R);
  std::cout << R << std::endl;
  return 0;
}
