//
// Created by leander on 2/12/25.
//

#pragma once
#include <stdint.h>

constexpr float MAX_SPEED = 4000.0f;

/**
 * @enum MotorDirection
 * @brief Enumeration for motor direction control.
 */
enum MotorDirection
{
    FORWARD, ///< Motor moves forward
    BACKWARD, ///< Motor moves backward
};

/**
 * @brief Sets the direction of the motor.
 *
 * @param dac The DAC index to update.
 * @param direction The direction to set (FORWARD or BACKWARD).
 */
void setMotorDirection(uint8_t dac, MotorDirection direction);

uint8_t getDirectionPin(uint8_t dac);

/**
 * @brief Gets the DAC pin associated with a given DAC index.
 *
 * @param dac The DAC index (0, 1, or 2).
 * @return The corresponding pin.
 */
uint8_t getDACPins(uint8_t dac);

/**
 * @brief Writes a 12-bit value to the specified DAC over SPI.
 *
 * @param dac The DAC index to update:
 *            - 0: Chip select on PA0
 *            - 1: Chip select on PA2
 *            - 2: Chip select on PA3
 * @param value The 12-bit value to write to the DAC (0-4095).
 */
void writeDAC(uint8_t dac, uint16_t value);

/**
 * @brief Controls motor speed using DAC output.
 *
 * @param dac The DAC channel to update.
 * @param speed Signed speed value (-4000 to +4000), where:
 *              - Negative values indicate backward motion
 *              - Positive values indicate forward motion
 */
void setMotorSpeedDAC(uint8_t dac, int speed);
