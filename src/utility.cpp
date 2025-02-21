//
// Created by tobias on 2/13/25.
//

#include "utility.h"

#include <Arduino.h>

int remap(const int value, const int fromLow, const int fromHigh, const int toLow, const int toHigh)
{
    if (fromLow == fromHigh) return toLow; // Prevent division by zero
    return constrain(static_cast<int>((value - fromLow) * (static_cast<float>(toHigh - toLow) / (fromHigh - fromLow)) + toLow), toLow, toHigh);
}

float remap(const float value, const float fromLow, const float fromHigh, const float toLow, const float toHigh)
{
    if (fromLow == fromHigh) return toLow; // Prevent division by zero
    return constrain((value - fromLow) * ((toHigh - toLow) / (fromHigh - fromLow)) + toLow, toLow, toHigh);
}

float normalize(const float value, const float min, const float max)
{
    if (min == max) return 0.0f; // Prevent division by zero
    return constrain((value - min) / (max - min), 0.0f, 1.0f);
}