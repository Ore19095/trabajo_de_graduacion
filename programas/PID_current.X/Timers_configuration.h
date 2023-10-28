/* 
 * File: Timer_configuration.h
 * Author: Angel Orellana
 * Comments: Funciones para la configuración de timers en ATMEGA328P
 * Revision history: 
 */

#ifndef TIMERS_CONFIG_E_H

#define	TIMERS_CONFIG_E_H
#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

//-- CONSTANTES A MODIFICAR PARA LA FRECUENCIA DE TIMER 2 ---------
#define TIMER2_FREQ 10000L
#define F_CPU 8000000L
//-- CONSTANTES A MODIFICAR PARA LA FRECUENCIA DE TIMER 0 ---------
#define TIMER0_FREQ 10000L
//-----------------------------------------------------------------
#ifndef OCR2A_VALUE 
#define OCR2A_VALUE(x) F_CPU /(x * TIMER2_FREQ) - 1L
#endif
/* En esta secci�n se determina el valor necesario para el preescalador
   en base a la frecuencia que se desea alcanzar*/
#define TIMER2_PREESCALER 1024L /*Valor del prescalador para el timer 2*/

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 256L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 128L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 64L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 32L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 16L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 8L
#endif

#if OCR2A_VALUE(TIMER2_PREESCALER) < 0
#define TIMER2_PREESCALER 1L
#endif

#if  OCR2A_VALUE(TIMER2_PREESCALER) < 0
#error "no es posible alcanzar la frecuencia solicitada"
#endif
//------------------------------------------------------------------

/*En esta sección se determina el valor que deberia tener
   OCR0A*/
void configure_timer2();
void configure_pwm_timer1();
void configure_pwm_timer0();

#endif	/* XC_HEADER_TEMPLATE_H */

