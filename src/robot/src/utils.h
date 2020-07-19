#ifndef utils_h
#define utils_h


// Utility for getting sign of values
// From: https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float wrap_angle(float a);

float angle_diff(float a1, float a2);

// Make it easier to switch from arduino
// TODO: Remove this
unsigned long millis();

#endif
