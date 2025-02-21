//
// Created by tobias on 2/12/25.
//
#include "motor.h"
#include <Arduino.h>

uint8_t getDirectionPin(const uint8_t dac)
{
    switch (dac)
    {
    case 0:
        return 11;
    case 1:
        return 12;
    case 2:
        return 13;
    default:
        return 11;
    }
}

void setMotorDirection(const uint8_t dac, const MotorDirection direction)
{
    const auto pin = getDirectionPin(dac);
    if (direction == FORWARD)
    {
        Serial.println("Forward");
        Serial.println(pin);
        digitalWrite(pin, LOW);
        return;
    }

    
    digitalWrite(pin, HIGH);
}

uint8_t getDACPins(const uint8_t dac)
{
    switch (dac)
    {
    case 0:
        return PA0;
    case 1:
        return PA2;
    case 2:
        return PA3;
    default:
        return PA0;
    }
}

void writeDAC(const uint8_t dac, uint16_t value)
{
    value = constrain(value, 0, 4095);

    const uint8_t csPin = getDACPins(dac);

    // Begin transmission to the selected DAC:
    PORTA &= ~(1 << csPin);

    // Send the first byte (the upper 4 bits).
    SPDR = (0b00110000) | ((value >> 8) & 0x0F);
    while (!(SPSR & 1 << SPIF))
    {
    } // Wait for transmission to complete.

    // Send the second byte (the lower 8 bits).
    SPDR = value & 0xFF;
    while (!(SPSR & 1 << SPIF))
    {
    } // Wait for transmission to complete.

    // End transmission by raising the chip-select pin.
    PORTA |= 1 << csPin;

    // Pulse the common update/latch pin on PA1.
    PORTA &= ~(1 << PA1);
    PORTA |= 1 << PA1;
}

void setMotorSpeedDAC(const uint8_t dac, int speed)
{
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
    writeDAC(dac, abs(speed));
    setMotorDirection(dac, speed >= 0 ? FORWARD : BACKWARD);
}
