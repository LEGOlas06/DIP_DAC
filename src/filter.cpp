//
// Created by leander on 2/12/25.
//
#include "filter.h"

MovingAverage::MovingAverage(const int size): windowSize(size), bufferIndex(0), initialized(false), sum(0)
{
    buffer = new int[windowSize]();
}

MovingAverage::~MovingAverage()
{
    delete[] buffer;
}

void MovingAverage::addValue(const int newValue) {
    if (!initialized) {
        for (int i = 0; i < windowSize; i++) {
            buffer[i] = newValue;
            sum += newValue;
        }
        initialized = true;
    } else {
        sum -= buffer[bufferIndex];
        buffer[bufferIndex] = newValue;
        sum += newValue;
    }
    bufferIndex = (bufferIndex + 1) % windowSize;
}

int MovingAverage::updateValue(const int newValue)
{
    addValue(newValue);
    return getAverage();
}

int MovingAverage::getAverage() const
{
    return sum / windowSize;
}
