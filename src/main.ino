#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "lib/lcdarduino.h"
#include <util/delay.h>

/*
Pin-Zuweisungen:
- PA0: DAC Chip Select
- PA1: Latch
- PB0: Chip Select
- PB2: MOSI
*/

// Menü- und LCD-Steuerung
volatile unsigned char menuStatus = 1;
volatile unsigned char previousMenuStatus = 0;
char displayBuffer[16];
volatile char debounceFlag = 0;
volatile char buttonLock = 0;

// Signalparameter
volatile float amplitude = 2048;  // Amplitude des Dreieckssignals
volatile float frequency = 1000;  // Frequenz des Dreieckssignals
volatile float updatedAmplitude = 2048;
volatile float updatedFrequency = 1000;
volatile int signalValue = 0;
volatile bool increasing = true;

// Timer0 Compare Match A Interrupt für den Abtastzyklus
ISR (TIMER0_COMPA_vect) {
    // Dreieckssignal erzeugen
    if (increasing) {
        signalValue += (int)(updatedAmplitude * 2 * updatedFrequency / 1000);
        if (signalValue >= updatedAmplitude) {
            signalValue = updatedAmplitude;
            increasing = false;
        }
    } else {
        signalValue -= (int)(updatedAmplitude * 2 * updatedFrequency / 1000);
        if (signalValue <= 0) {
            signalValue = 0;
            increasing = true;
        }
    }

    // Übertragung des Ausgangswertes zum DAC
    PORTA &= ~(1 << PA0);  // DAC Chip Select aktivieren (PA0)
    SPDR = (0b00010000) | ((signalValue >> 8) & 0x0F);  // Oberen 4 Bits senden
    while (!(SPSR & (1 << SPIF)));
    SPDR = signalValue & 0xFF;  // Unteren 8 Bits senden
    while (!(SPSR & (1 << SPIF)));
    PORTA |= (1 << PA0);  // DAC Chip Select deaktivieren (PA0)

    // Latch-Ausgang setzen und zurücksetzen
    PORTA &= ~(1 << PA1);  // Latch aktivieren (PA1)
    PORTA |= (1 << PA1);   // Latch deaktivieren (PA1)
}

void setup() {
    // Initialisierung der Ports
    DDRB = 0x07;  // PB0, PB1, PB2 als Ausgang
    DDRA = 0x03;  // PA0, PA1 als Ausgang

    // SPI-Konfiguration für Master-Modus
    SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << SPR1);
    SPSR = 0;

    // Timer0-Konfiguration für 1 ms Abtastrate
    TCCR0A = 2 << WGM00;  // CTC Modus
    TCCR0B = 3 << CS00;   // Prescaler=64
    OCR0A = 249;          // Zählerwert für 1 ms
    TIMSK0 = 1 << OCIE0A; // Interrupt aktivieren

    // Globale Interrupts aktivieren
    sei();

    // Chip Select auf Ausgang high setzen
    PORTB |= (1 << PB0);  // Chip Select (PB0)
    PORTA |= (1 << PA0);  // DAC Chip Select (PA0)
    PORTA |= (1 << PA1);  // Latch (PA1)
    PORTB &= ~(1 << PB2); // MOSI initialisieren (PB2)

    // LCD initialisieren und Startwerte anzeigen
    lcd_init(LCD_DISP_ON_CURSOR);
    lcd_clrscr();
    sprintf(displayBuffer, "Frq: %d Hz", int(frequency));
    lcd_puts(displayBuffer);
    sprintf(displayBuffer, "\nAmp: %d", int(amplitude));
    lcd_puts(displayBuffer);
}

void loop() {
    lcd_home();

    // Tastenabfrage zur Steuerung von Frequenz und Amplitude
    char buttonPress = get_button();
    switch (buttonPress) {
        case button_up:
            if (buttonLock == 0) {
                amplitude += 100; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_down:
            if (buttonLock == 0) {
                amplitude -= 100; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_left:
            if (buttonLock == 0) {
                frequency -= 100; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_right:
            if (buttonLock == 0) {
                frequency += 100; buttonLock = 1; previousMenuStatus = 0;
            } break;
        case button_ok:
            if (buttonLock == 0) {
                buttonLock = 1; previousMenuStatus = 2;
            } break;
        default: buttonLock = 0; break;
    }

    // LCD-Anzeige aktualisieren, wenn Menü geändert wurde
    if (menuStatus != previousMenuStatus) {
        if (previousMenuStatus == 0) {
            sprintf(displayBuffer, "Frq: %d Hz", int(frequency));
            lcd_puts(displayBuffer);
            sprintf(displayBuffer, "\nAmp: %d", int(amplitude));
            lcd_puts(displayBuffer);
            previousMenuStatus = 1;
        }

        // Aktualisierte Signalparameter anwenden
        if (previousMenuStatus == 2) {
            updatedFrequency = frequency;
            updatedAmplitude = amplitude;
            previousMenuStatus = 1;
        }
    }
}