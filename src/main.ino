#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// ADC-Daten
volatile uint16_t adcHighByte = 0;
volatile uint16_t adcLowByte = 0;

// Menü- und LCD-Steuerung
volatile unsigned char menuStatus = 1;
volatile unsigned char previousMenuStatus = 0;
char displayBuffer[16];
volatile char debounceFlag = 0;
volatile char buttonLock = 0;

// Dreieckssignal-Parameter
volatile uint16_t triangleSignal1 = 0;
volatile uint16_t triangleSignal2 = 2048; // 180 Grad Phasenverschiebung
volatile double triangleStep = 1;
volatile uint16_t maxSignalValue = 4095;

// Filterparameter
volatile double filterFrequency = 1; // Beispielinitialwert
volatile double filterGain = 1;         // Beispielinitialwert

// Aktualisierte Filterparameter
volatile double updatedFrequency = 1; // Beispielinitialwert
volatile double updatedGain = 1;         // Beispielinitialwert

// Filterkoeffizienten
volatile float a0 = 0.0;
volatile float b1 = 0.0;

// Ausgangswerte
volatile float previousOutput = 0.0;
volatile float currentOutput = 0.0;

const int numReadings = 10; // Number of readings for averaging
unsigned long readings[numReadings]; // Array to store readings
int readIndex = 0; // Current index in the array
unsigned long total = 0; // Sum of readings
unsigned long average = 0; // Average value

// SPI-Interrupt-Service-Routine
ISR(SPI_STC_vect) {
    // Hier könnte Code hinzugefügt werden, um auf SPI-Interrupts zu reagieren
}

// Funktion zum Lesen des analogen Werts von PK0
int readAnalogPK0() {
    ADCSRA |= (1 << ADSC); // Start der ADC-Konvertierung
    while (ADCSRA & (1 << ADSC)); // Warten, bis die Konvertierung abgeschlossen ist
    return ADC;
}

unsigned long customPulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
    unsigned long width = 0;
    unsigned long start = micros();

    // Warte, bis der Pin den gewünschten Zustand erreicht
    while (digitalRead(pin) != state) {
        if (micros() - start > timeout) {
            return 0; // Timeout erreicht
        }
    }

    // Startzeit des Pulses
    unsigned long pulseStart = micros();

    // Warte, bis der Pin den gegenteiligen Zustand erreicht
    while (digitalRead(pin) == state) {
        if (micros() - start > timeout) {
            return 0; // Timeout erreicht
        }
    }

    // Endzeit des Pulses
    unsigned long pulseEnd = micros();

    // Berechne die Pulsbreite
    width = pulseEnd - pulseStart;

    return width;
}

// Timer0 Compare Match A Interrupt für den Abtastzyklus
ISR (TIMER0_COMPA_vect) {
    // Dreieckssignal 1 berechnen
    triangleSignal1 += triangleStep * filterGain; // Amplitude berücksichtigen
    if (triangleSignal1 >= maxSignalValue) {
        triangleSignal1 = maxSignalValue;
        triangleStep = -triangleStep;
    } else if (triangleSignal1 <= 0) {
        triangleSignal1 = 0;
        triangleStep = -triangleStep;
    }

    // Dreieckssignal 2 berechnen (180 Grad Phasenverschiebung)
    triangleSignal2 -= triangleStep * filterGain; // Amplitude berücksichtigen
    if (triangleSignal2 >= maxSignalValue) {
        triangleSignal2 = maxSignalValue;
    } else if (triangleSignal2 <= 0) {
        triangleSignal2 = 0;
    }

    // Lesen des analogen Werts von PK0
    uint16_t analogValue = readAnalogPK0();

    // Übertragung des analogen Werts von PK0 zum ersten DAC
    PORTA &= ~(1 << PA0);  // DAC1 Chip Select aktivieren
    SPDR = (0b00010000) | ((analogValue >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = analogValue & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA0);  // DAC1 Chip Select deaktivieren

    // Übertragung des Dreieckssignals 1 zum zweiten DAC
    PORTA &= ~(1 << PA2);  // DAC2 Chip Select aktivieren
    SPDR = (0b00010000) | ((triangleSignal1 >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = triangleSignal1 & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA2);  // DAC2 Chip Select deaktivieren



    // Latch-Ausgang setzen und zurücksetzen
    PORTA &= ~(1 << PA1);
    PORTA |= (1 << PA1);
}

void setup() {
    Serial.begin(9600);
    pinMode(2, INPUT);

    // Initialize the readings array to 0
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
    }

    // Initialisierung der Ports
    DDRB = 0x07; // Setze PB0, PB1, PB2 als Ausgänge
    DDRA = 0x0F; // Setze PA0, PA1, PA2 und PA3 als Ausgänge
    DDRK &= ~(1 << PK0); // Setze PK0 als Eingang

    // SPI-Konfiguration für Master-Modus
    SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR1);
    SPSR = 0;

    // ADC-Konfiguration
    ADMUX = (1 << REFS0) | (1 << MUX2) | (1 << MUX0); // AVcc als Referenz, Kanal ADC8 (PK0)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC aktivieren, Prescaler=128

    // Timer0-Konfiguration für 1 ms Abtastrate
    TCCR0A = (1 << WGM01);  // CTC Modus
    TCCR0B = (1 << CS01) | (1 << CS00);   // Prescaler=64
    OCR0A = 249;          // Zählerwert für 1 ms
    TIMSK0 = (1 << OCIE0A); // Interrupt aktivieren

    // Globale Interrupts aktivieren
    sei();

    // Chip Select auf Ausgang high setzen
    PORTB |= (1 << PB0);
    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    PORTA |= (1 << PA2);
    PORTA |= (1 << PA3);
    PORTB &= ~(1 << PB2); // MOSI initialisieren

    // Dreieckssignal-Parameter anpassen
    triangleStep = 1; // Reduziere den Schritt um den Faktor 10
}

void loop() {
    // Temporäre Variablen für die Eingabe
    static double tempFrequency = filterFrequency;
    static double tempGain = filterGain;

    // Aktualisierte Filterparameter anwenden
    updatedFrequency = filterFrequency;
    updatedGain = filterGain;

    // Berechnen der neuen Tiefpass-Koeffizienten
    a0 = (1 / updatedFrequency) / (0.001 + 1 / updatedFrequency);
    b1 = (0.001) / (0.001 + 1 / updatedFrequency);

    previousOutput = currentOutput;

    // Timer0-Konfiguration für die neue Frequenz
    TCCR0B = 3 << CS00;   // Prescaler=64
    OCR0A = (16000000 / (64 * filterFrequency)) - 1; // Zählerwert für die neue Frequenz

    // Subtract the last reading from the total
    total = total - readings[readIndex];

    // Take a new reading
    readings[readIndex] = pulseIn(2, LOW, 3000000); // Use a smaller value

    // Add the new reading to the total
    total = total + readings[readIndex];

    // Advance to the next position in the array
    readIndex = readIndex + 1;

    // If we're at the end of the array, wrap around to the beginning
    if (readIndex >= numReadings) {
        readIndex = 0;
    }

    // Calculate the average
    average = (total / numReadings - 6700) * 7.8;

    if (average <= 6) {
        average = 0;
    }

    // Print the average duration
    Serial.println(average);

    // Übertragung des average-Signals zum dritten DAC
    PORTA &= ~(1 << PA3);  // DAC3 Chip Select aktivieren
    SPDR = (0b00010000) | ((average >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = average & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA3);  // DAC3 Chip Select deaktivieren

}
