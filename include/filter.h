//
// Created by leander on 2/12/25.
//

#pragma once

class MovingAverage {
    const int windowSize;
    int *buffer;
    int bufferIndex;
    bool initialized;
    long sum;

public:
    MovingAverage() :  windowSize(20), bufferIndex(0), initialized(false), sum(0) {
        buffer = new int[windowSize]();
    }
    
    explicit MovingAverage(int size);
    ~MovingAverage();
    MovingAverage(const MovingAverage& other);
    
    void addValue(int newValue);
    int updateValue(int newValue);
    int getAverage() const;
};

