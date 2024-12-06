#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lib/lcdarduino.h"
#include <util/delay.h>

// ADC-Daten
volatile int adcHighByte = 0;
volatile int adcLowByte = 0;

// Menü- und LCD-Steuerung
volatile unsigned char menuStatus = 1;
volatile unsigned char previousMenuStatus = 0;
char displayBuffer[16];
volatile char debounceFlag = 0;
volatile char buttonLock = 0;

// Dreieckssignal-Parameter
volatile int triangleSignal1 = 0;
volatile int triangleSignal2 = 2048; // 180 Grad Phasenverschiebung
volatile int triangleStep = 1;
volatile int maxSignalValue = 4095;

// Filterparameter
volatile int filterFrequency = 1000; // Beispielinitialwert
volatile int filterGain = 1;         // Beispielinitialwert

// Aktualisierte Filterparameter
volatile int updatedFrequency = 1000; // Beispielinitialwert
volatile int updatedGain = 1;         // Beispielinitialwert

// Filterkoeffizienten
volatile float a0 = 0.0;
volatile float b1 = 0.0;

// Ausgangswerte
volatile float previousOutput = 0.0;
volatile float currentOutput = 0.0;

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
    int analogValue = readAnalogPK0();

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

    // Übertragung des Dreieckssignals 2 zum dritten DAC
    PORTA &= ~(1 << PA3);  // DAC3 Chip Select aktivieren
    SPDR = (0b00010000) | ((triangleSignal2 >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = triangleSignal2 & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA3);  // DAC3 Chip Select deaktivieren

    // Latch-Ausgang setzen und zurücksetzen
    PORTA &= ~(1 << PA1);
    PORTA |= (1 << PA1);
}

void setup() {
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

    // LCD initialisieren und Startwerte anzeigen
    lcd_init(LCD_DISP_ON_CURSOR);
    lcd_clrscr();
    sprintf(displayBuffer, "Frq: %d s-1", int(filterFrequency));
    lcd_puts(displayBuffer);
    sprintf(displayBuffer, "\nGain: %d", int(filterGain));
    lcd_puts(displayBuffer);
}

void loop() {
    lcd_home();

    // Temporäre Variablen für die Eingabe
    static int tempFrequency = filterFrequency;
    static int tempGain = filterGain;

    // Tastenabfrage zur Steuerung von Frequenz und Gain
    char buttonPress = get_button();
    switch (buttonPress) {
        case button_up:
            if (buttonLock == 0) {
                tempGain++; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_down:
            if (buttonLock == 0) {
                tempGain--; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_left:
            if (buttonLock == 0) {
                tempFrequency--; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_right:
            if (buttonLock == 0) {
                tempFrequency++; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_ok:
            if (buttonLock == 0) {
                filterFrequency = tempFrequency;
                if (tempGain < 0) tempGain = 0;
                filterGain = tempGain;
                buttonLock = 1; previousMenuStatus = 2;
            } break;
        default: buttonLock = 0; break;
    }

    // LCD-Anzeige aktualisieren, wenn Menü geändert wurde
    if (menuStatus != previousMenuStatus) {
        if (previousMenuStatus == 0) {
            sprintf(displayBuffer, "Frq: %d s-1", int(tempFrequency));
            lcd_puts(displayBuffer);
            sprintf(displayBuffer, "\nGain: %d", int(tempGain));
            lcd_puts(displayBuffer);
            previousMenuStatus = 1;
        }

        // Aktualisierte Filterparameter anwenden
        if (previousMenuStatus == 2) {
            updatedFrequency = filterFrequency;
            updatedGain = filterGain;

            // Berechnen der neuen Tiefpass-Koeffizienten
            a0 = (1 / updatedFrequency) / (0.001 + 1 / updatedFrequency);
            b1 = (0.001) / (0.001 + 1 / updatedFrequency);

            previousOutput = currentOutput;
            previousMenuStatus = 1;
        }
    }

    // Timer0-Konfiguration für die neue Frequenz
    TCCR0B = 3 << CS00;   // Prescaler=64
    OCR0A = (16000000 / (64 * filterFrequency)) - 1; // Zählerwert für die neue Frequenz
}