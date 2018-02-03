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
};

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
      Eigen::Matrix3d a;
      a << Eigen::MatrixXd::Identity(3, 3);
      return a;
    }
  else
    {
      MR::AxisWAngle omgthetasplit = MR::AxisAng3(omgtheta);
      double theta = omgthetasplit.angle; //3.74166
      Eigen::Matrix3d omgmat = so3mat / theta;
      //Eigen::MatrixXd::Identity(3, 3) + sin(theta) * omgmat;
      return Eigen::MatrixXd::Identity(3, 3) + (sin(theta) * omgmat) + (1 - cos(theta)) * (omgmat*omgmat);
    }
}

int main()
{
  std::cout << MR::NearZero(1e-7) << "\n";
  Eigen::Matrix3d V;
  V << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  std::cout << V << std::endl;
  V = MR::MatrixExp3(V);
  std::cout << V << std::endl;
  return 0;
}
