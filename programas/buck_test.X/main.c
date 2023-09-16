/*
 * File:   main.c
 * Author: AngelOrellana
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>


int main(void) {
        // ----     IO CONFIG  ---------
    DDRB = 0b00000011; // PB[7:2] input, PB1 out, PB0 out
    
    PORTB = 0b0000000; //activar PB0
    while (1) {
        PORTB |= 1<<1; //PB1 en 1
        _delay_ms(10000);
        PORTB &= ~(1<<1); //PB1 en 0
        _delay_ms(10000);
    }
}
