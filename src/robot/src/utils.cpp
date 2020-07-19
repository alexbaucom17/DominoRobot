#define _USE_MATH_DEFINES
 
#include <cmath>
#include "utils.h"
#include <chrono>

float wrap_angle(float a)
{
  if(a > M_PI)
  {
    a -= M_2_PI;
  }
  else if (a < -1*M_PI)
  {
    a += M_2_PI;
  }
  return a;
}

float angle_diff(float a1, float a2)
{
  float outA = a1 - a2;
  // Handle angle wrapping and compute the correct error amount
  outA = wrap_angle(outA);
  return outA;
}

unsigned long millis()
{
  using namespace std::chrono;
  milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
  return ms.count();
}