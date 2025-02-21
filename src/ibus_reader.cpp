//
// Created by tobias on 2/13/25.
//
#include "ibus_reader.h"

#include <Arduino.h>
#include <filter.h>
#include <utility.h>


IBusReader::IBusReader(const uint8_t rxPin, const uint8_t txPin)
    : serial(rxPin, txPin), ibusIndex(0)
{
    channelFilters = new MovingAverage[IBUS_MAXCHANNELS];
}

void IBusReader::begin(const long baudRate)
{
    serial.begin(baudRate);
}

void IBusReader::readRx()
{
    uint16_t rcValue[IBUS_MAXCHANNELS]{};
    while (serial.available())
    {
        const uint8_t val = serial.read();

        // Look for 0x2040 as start of packet
        if (ibusIndex == 0 && val != 0x20)
        {
            ibusIndex = 0;
            continue;
        }
        if (ibusIndex == 1 && val != 0x40)
        {
            ibusIndex = 0;
            continue;
        }

        ibus[ibusIndex++] = val;

        if (ibusIndex < IBUS_BUFFSIZE)
            continue;

        ibusIndex = 0;
        for (int i = 0; i < IBUS_MAXCHANNELS; i++)
        {
            const int high = 3 + i * 2;
            const int low = 2 + i * 2;
            rcValue[i] = (ibus[high] << 8) + ibus[low];

            if (rcValue[i] < 1000 || rcValue[i] > 2000)
            {
                continue;
            }

            const int remapped = channelFilters[i].updateValue(remap(static_cast<int>(rcValue[i]), 1000, 2000, 0, 4000));
            if (remapped > 4000)
            {
                continue;
            }
            
            remappedValues[i] = remapped;
        }
        break;
    }
}

float IBusReader::getChannelValue(const int channel) const
{
    if (channel >= 0 && channel < IBUS_MAXCHANNELS)
    {
        const auto v = normalize(remappedValues[channel], 0.0f, 4000.0f);
        return v;
    }
    return 0;
}
