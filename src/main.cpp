#include <Arduino.h>
#include <ibus_reader.h>
#include <motor.h>
#include <Servo.h>
#include <utility.h>
#include <avr/interrupt.h>
#include <avr/io.h>

Servo ch2;

IBusReader reader(10, 11);

void setup()
{
    Serial.begin(115200);
    reader.begin();

    pinMode(11,OUTPUT);
    pinMode(12,OUTPUT);
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);

    // Initialize Ports
    DDRB = 0x07; // Set PB0, PB1, PB2 as outputs
    DDRA = 0x0F; // Set PA0, PA1, PA2, and PA3 as outputs
    DDRK &= ~(1 << PK0); // Set PK0 as input

    // SPI Configuration for Master Mode
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // Removed SPIE
    SPSR = 0;

    // ADC Configuration
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX0); // AVcc as reference, Channel ADC8 (PK0)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, Prescaler=128

    // Timer1 Configuration for 1 ms Sampling Rate
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler=64
    OCR1A = 249; // For 1 ms at 16 MHz: (16,000,000 / 64 / 1000) - 1 = 249
    TIMSK1 = (1 << OCIE1A); // Enable the compare match interrupt

    // Set Chip Select to high
    PORTB |= (1 << PB0);
    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    PORTA |= (1 << PA2);
    PORTA |= (1 << PA3);
    PORTB &= ~(1 << PB2); // Initialize MOSI

    // Removed old interrupt attachments
    // attachInterrupt(0, messung, CHANGE); // Removed

    pinMode(10,INPUT);

    // Enable global interrupts
    sei();
}

int getChannel(const int channel)
{
    return static_cast<int>(remap(
        reader.getChannelValue(channel),
        0.0f,
        1.0f,
        -MAX_SPEED,
        MAX_SPEED
    ));
}

void setMovement(const int linearSpeed, const int angularSpeed)
{
    const auto leftMotorSpeed = linearSpeed - angularSpeed;
    const auto rightMotorSpeed = linearSpeed + angularSpeed;
    setMotorSpeedDAC(0, rightMotorSpeed);
    setMotorSpeedDAC(1, leftMotorSpeed);
}

void loop()
{
    reader.readRx();

    const auto ch1 = getChannel(0); // Forward/Backward
    const auto ch2 = getChannel(1); // Left/Right
    const auto ch3 = getChannel(2); // Up/Down
    
    setMovement(ch1, ch2);
    //setMotorSpeedDAC(0, ch3);
}
