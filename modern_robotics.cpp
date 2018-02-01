#include "modern_robotics.hpp"
#include <math.h>
#include <iostream>

bool NearZero(double z)
{
  return fabs(z) < 1e-6;
}

int main()
{
  std::cout << NearZero(1e-7) << "\n";
  return 0;
}
