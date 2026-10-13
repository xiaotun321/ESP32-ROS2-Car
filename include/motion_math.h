#pragma once
#include <math.h>
namespace motion {
inline bool twistToPwm(double linear, double angular, float linearGain,
                       float angularGain, float &left, float &right) {
    if (!isfinite(linear) || !isfinite(angular) || fabs(linear) > 10 || fabs(angular) > 20)
        return false;
    left = linear * linearGain - angular * angularGain;
    right = linear * linearGain + angular * angularGain;
    return true;
}
inline void limitPair(float &left, float &right, float limit) {
    const float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > limit) { left *= limit / peak; right *= limit / peak; }
}
}
