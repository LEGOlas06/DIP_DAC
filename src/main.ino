#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// ADC Data
volatile uint16_t adcHighByte = 0;
volatile uint16_t adcLowByte = 0;

// Menu and LCD Control
volatile unsigned char menuStatus = 1;
volatile unsigned char previousMenuStatus = 0;
char displayBuffer[16];
volatile char debounceFlag = 0;
volatile char buttonLock = 0;

// Triangle Signal Parameters
volatile uint16_t triangleSignal1 = 0;
volatile uint16_t triangleSignal2 = 2048; // 180 degrees phase shift
volatile double triangleStep = 1;
volatile uint16_t maxSignalValue = 4095;

// Filter Parameters
volatile double filterFrequency = 1; // Example initial value
volatile double filterGain = 1; // Example initial value

// Updated Filter Parameters
volatile double updatedFrequency = 1; // Example initial value
volatile double updatedGain = 1; // Example initial value

// Filter Coefficients
volatile float a0 = 0.0;
volatile float b1 = 0.0;

// Output Values
volatile float previousOutput = 0.0;
volatile float currentOutput = 0.0;

// Readings for Averaging
const int numReadings = 10; // Number of readings for averaging
unsigned long readings[numReadings]; // Array to store readings
int readIndex = 0; // Current index in the array

// New iBus Reading Variables and Definitions
#define IBUS_BUFFSIZE 32
#define IBUS_MAXCHANNELS 10

static uint8_t ibusIndex = 0;
static uint8_t ibus[IBUS_BUFFSIZE] = {0};
static uint16_t rcValue[IBUS_MAXCHANNELS];

static boolean rxFrameDone;

int ch_width_1;
int ch_width_2;
int ch_width_3;
int ch_width_4;
int ch_width_5;
int ch_width_6;
int ch_width_7;
int ch_width_8;
int ch_width_9;
int ch_width_10;

Servo ch2;  // Servo on pin 2

// Define SoftwareSerial RX and TX pins
SoftwareSerial mySerial(10, 11);  // RX = pin 10, TX = pin 11
int remapped;

// Moving average buffer for remapped value
const int MOVING_WINDOW = 10;
int remappedBuffer[MOVING_WINDOW] = {0};
int remappedBufferIndex = 0;
bool remappedBufferInitialized = false;

void readRx() {
    rxFrameDone = false;

    while (1) {
        if (mySerial.available()) {
            uint8_t val = mySerial.read();
            // Look for 0x2040 as start of packet
            if (ibusIndex == 0 && val != 0x20) {
                ibusIndex = 0;
                continue;
            }
            if (ibusIndex == 1 && val != 0x40) {
                ibusIndex = 0;
                continue;
            }

            if (ibusIndex == IBUS_BUFFSIZE) {
                ibusIndex = 0;
                int high = 3;
                int low = 2;
                for (int i = 0; i < IBUS_MAXCHANNELS; i++) {
                    rcValue[i] = (ibus[high] << 8) + ibus[low];
                    high += 2;
                    low += 2;
                }

                ch_width_1 = map(rcValue[0], 1000, 2000, 1000, 2000);
                ch2.writeMicroseconds(ch_width_1);

                ch_width_2 = map(rcValue[1], 1000, 2000, 1000, 2000);

                ch_width_3 = map(rcValue[2], 1000, 2000, 1000, 2000);
                ch_width_4 = map(rcValue[3], 1000, 2000, 1000, 2000);
                ch_width_5 = map(rcValue[4], 1000, 2000, 1000, 2000);
                ch_width_6 = map(rcValue[5], 1000, 2000, 1000, 2000);
                ch_width_7 = map(rcValue[6], 1000, 2000, 1000, 2000);
                ch_width_8 = map(rcValue[7], 1000, 2000, 1000, 2000);
                ch_width_9 = map(rcValue[8], 1000, 2000, 1000, 2000);
                ch_width_10 = map(rcValue[9], 1000, 2000, 1000, 2000);

                if (ch_width_1 > 2000 || ch_width_1 < 900) {
                    // Serial.println("Invalid value - Too high / Too Low");
                    break;
                }

                int newRemapped = map(ch_width_1, 1000, 2000, 0, 4096);
                if (newRemapped < 0 || newRemapped > 4000) {
                    // Serial.println("Invalid value - Out of range");
                    break;
                }

                // Update moving average buffer for remapped value
                if (!remappedBufferInitialized) {
                    // On the first valid reading, fill the entire buffer
                    for (int i = 0; i < MOVING_WINDOW; i++) {
                        remappedBuffer[i] = newRemapped;
                    }
                    remappedBufferInitialized = true;
                } else {
                    // Place the new value in the current buffer position
                    remappedBuffer[remappedBufferIndex] = newRemapped;
                }
                // Increment and wrap the index
                remappedBufferIndex = (remappedBufferIndex + 1) % MOVING_WINDOW;

                // Compute the average of the last 5 values
                long sum = 0;
                for (int i = 0; i < MOVING_WINDOW; i++) {
                    sum += remappedBuffer[i];
                }
                remapped = sum / MOVING_WINDOW;

                rxFrameDone = true;
                break;
            } else {
                ibus[ibusIndex] = val;
                ibusIndex++;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);

    Serial.println("Setup started");
    mySerial.begin(115200);  // Initialize SoftwareSerial for iBus
    ch2.attach(2); // Attach servo to pin 2

    // Initialize Ports
    Serial.println("Init ports");
    DDRB = 0x07; // Set PB0, PB1, PB2 as outputs
    DDRA = 0x0F; // Set PA0, PA1, PA2, and PA3 as outputs
    DDRK &= ~(1 << PK0); // Set PK0 as input

    // SPI Configuration for Master Mode
    Serial.println("SPI config");
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); // Removed SPIE
    SPSR = 0;

    // ADC Configuration
    Serial.println("ADC config");
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX0); // AVcc as reference, Channel ADC8 (PK0)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, Prescaler=128

    // Timer1 Configuration for 1 ms Sampling Rate
    Serial.println("Timer1 config");
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler=64
    OCR1A = 249; // For 1 ms at 16 MHz: (16,000,000 / 64 / 1000) - 1 = 249
    TIMSK1 = (1 << OCIE1A); // Enable the compare match interrupt

    // Set Chip Select to high
    Serial.println("Chip select (ig SPI?)");
    PORTB |= (1 << PB0);
    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    PORTA |= (1 << PA2);
    PORTA |= (1 << PA3);
    PORTB &= ~(1 << PB2); // Initialize MOSI

    // Removed old interrupt attachments
    // attachInterrupt(0, messung, CHANGE); // Removed

    // Enable global interrupts
    sei();
    Serial.println("Setup completed");
}

void loop() {
    readRx(); // Read data using the new method

    // Send filteredHighTime (ch_width_3) to DAC1
    Serial.println(remapped);
    PORTA &= ~(1 << PA3); // DAC1 Chip Select aktivieren
    SPDR = (0b00110000) | ((remapped >> 8) & 0x0F); // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = remapped & 0xFF; // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA3); // DAC1 Chip Select deaktivieren

    // Send filteredHighTime (ch_width_3) to DAC3
    Serial.println(remapped);
    PORTA &= ~(1 << PA3); // DAC3 Chip Select aktivieren
    SPDR = (0b00110000) | ((remapped >> 8) & 0x0F); // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = remapped & 0xFF; // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA3); // DAC3 Chip Select deaktivieren
}