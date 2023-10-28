/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 20, 2023, 7:03 PM
 */

// ---------------   FUSES    ---------------------------


//---------------- CONSTANTES --------------------------

#define F_CPU 8000000L

#include <xc.h>
#include <stdint.h>
#include <util/delay.h>
#include "Timers_configuration.h"
// --------------- VARIABLES -----------------------------

int main(void) {
    
    configure_pwm_timer2();
    configure_timer1();
    configure_timer0();

    return 0;
}

// --------------- FUNCIONES -----------------------------


