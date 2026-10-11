#pragma once
#include <math.h>
namespace motion {
inline void limitPair(float &left, float &right, float limit) {
    const float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > limit) { left *= limit / peak; right *= limit / peak; }
}
}
