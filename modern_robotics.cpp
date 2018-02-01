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


int main()
{
  std::cout << MR::NearZero(1e-7) << "\n";
  Eigen::Vector3d V(1,2,3);
  Eigen::Matrix3d mat;
  mat << 0, -3, 2, 3, 0 , -1, -2, 1 , 0;
  //mat = MR::VecToso3(V);
  V = MR::so3ToVec(mat);
  std::cout << V << std::endl;
  return 0;
}
