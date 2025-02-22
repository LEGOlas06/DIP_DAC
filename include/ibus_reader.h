//
// Created by leander on 2/13/25.
//

#pragma once

#include <filter.h>
#include <SoftwareSerial.h>
#include <Servo.h>

constexpr int IBUS_BUFFSIZE = 32;
constexpr int IBUS_MAXCHANNELS = 10;

class IBusReader
{
public:
    IBusReader(uint8_t rxPin, uint8_t txPin);
    void begin(long baudRate = 115200);
    void readRx();
    float getChannelValue(int channel) const;

private:
    SoftwareSerial serial;
    uint8_t ibusIndex;

    uint8_t ibus[IBUS_BUFFSIZE]{};
    uint16_t remappedValues[IBUS_MAXCHANNELS]{};

    MovingAverage* channelFilters;
};