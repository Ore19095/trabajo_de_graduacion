/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 20, 2023, 7:03 PM
 */

// ---------------   FUSES    ---------------------------


//---------------- CONSTANTES --------------------------

#define F_CPU 8000000UL


#include <xc.h>
#include <stdint.h>
#include <util/delay.h>
// --------------- VARIABLES -----------------------------




int main(void) {

  //--------- CONFIGURACION DE PUERTOS --------
  DDRB = 0b11111111; //PB0-7 como salidas
  DDRD = 0b11110010; //PD4-7,PD1 como salidas, PD2-3,PD0 como entrada
  // Colocar PD4 como salida
  PORTD &= ~(1<<PD4); //type:ignore 

  //---------   CONFIGURACION PWM 0    -------- 

  /*modulo timer 0 con un preescalador de 64, generando un pwm de 
    50%, unicamente el pin OC0A sera como salida, el pin OC0B 
    estará en modo normal (no pwm). Se configuró el modulo como 
    fast pwm, siendo OC0A puesto en 1 al llegar al valor bottom, y 
    puesto en 0 cuando coincida con el valor del registro OCR0A. El valor
    TOP para este modulo se definirá  */

  // WGM0[1:0] = 0b11 fast pwm, TOP = OCRA
  TCCR0A |= (1<<WGM01) | (1<<WGM00);

  // COM0A[1:0] = 0b10, COM0B[1:0] = 0b00
  TCCR0A |= (1<<COM0A0); // OC0A en modo pwm, toggle on compare match
  TCCR0A &= ~(1<<COM0A1);
  TCCR0A &= ~(1<<COM0B1 | 1<<COM0B0); // OC0B en modo normal

  // CS0[2:0] = 0b011, preescalador = 64
  TCCR0B |= (1<<CS01) |  (1<<CS00) ;
  TCCR0B &= ~(1<<CS02);
  //WGM02 = 0b1 fast pwm, TOP = OCRA
  TCCR0B |= (1<<WGM02);

  // OCR0A = 7, duty cycle = 50%, Fpwm = 7812.5 Hz
  OCR0A = 7;

  //-------- CONFIGURACION PWM 1 ---------
  /* modulo de timer 1, sin preescalador, generará una señal pwm
      variable, con una resolución de 10 bits, la señal pwm se genera
      en el pin OC1A, el pin OC1B estará en modo normal (no pwm), se 
      configuró el modulo como fast pwm de 10 bits */

  // COM1A[1:0] = 0b10, COM1B[1:0] = 0b00, WGM1[3:0] = 0b1110
  TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10);
  TCCR1A &= ~(1<<COM1A0);

  // WGM1[3:2] = 0b01, CS1[2:0] = 0b001
  TCCR1B |= (1<<WGM12) | (1<<CS10);

  OCR1A = 1023; // duty cycle = 50% 
  //-------- CONFIGURACION ADC -----------

  /*ADC configurado para tener una frecuencia de operacion de 
    500kHz, con referencia el voltaje en el pin AVCC (5V). El 
    trigger para el ADC se configuró de forma que la conversion
    comience al momento que exista match en el contador 1 y OCRA. La
    interrupcion de fin de conversión se encuentra habilitada, para
    almacenar los datos inmediatamente despues de que termine la
    conversión*/

  ADMUX |= (1<<REFS0); // REFS[1:0] AVCC como referencia
  ADMUX &= ~(1<<REFS1);

  ADCSRA |= (1<<ADEN); // ADC habilitado
  ADCSRA |= (1<<ADIE); // Interrupcion de fin de conversion habilitada
  //preescalador = 16
  ADCSRA |= (1<<ADPS2);
  ADCSRA &= ~(1<<ADPS1 | 1<<ADPS0);

  ADCSRB |= (1<<ADTS1) | (1<<ADTS0)  ; // ADC trigger en match OC1A
  ADCSRB &= ~(1<<ADTS2);


  while(1){
    OCR1A = 1023; // duty cycle = 50%
    _delay_ms(2000);
    OCR1A = 0; // duty cycle = 0
    _delay_ms(2000);
  }



  return 0;
}
