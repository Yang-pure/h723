#include "clamp_symmetric.h"

float clamp_symmetric(float value, float limit)
{
    if (value > limit)
        return limit;
    if (value < -limit)
        return -limit;
    return value;
}
