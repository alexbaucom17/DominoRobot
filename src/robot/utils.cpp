#include "utils.h"
#include <arduino.h> // Needed for PI

float wrap_angle(float a)
{
  if(a > PI)
  {
    a -= TWO_PI;
  }
  else if (a < -1*PI)
  {
    a += TWO_PI;
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
